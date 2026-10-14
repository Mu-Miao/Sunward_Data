#!/usr/bin/env python3
"""macOS/Linux serial recorder, using only Python's standard library.

Opening a real port is performed only when the user launches the tool.
--self-test uses a software pseudo terminal, never the car's USB port.
"""
import argparse
from array import array
import datetime
import errno
import fcntl
import glob
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import termios
import time
from zoneinfo import ZoneInfo

ZONE = ZoneInfo('Asia/Shanghai')


def configure(fd):
    previous = termios.tcgetattr(fd)
    settings = termios.tcgetattr(fd)
    settings[0] = 0
    settings[1] = 0
    settings[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    settings[3] = 0
    settings[4] = settings[5] = termios.B115200
    settings[6][termios.VMIN] = 0
    settings[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, settings)
    return previous


def release_boot_controls(fd, ioctl=None):
    """C10B: deassert modem outputs so active-low RTS does not raise BOOT0.

    Driver bits are logical assertions, not measured voltages. Verify readback;
    never simulate this ioctl on the user's real port during --self-test.
    """
    ioctl = fcntl.ioctl if ioctl is None else ioctl
    mask = termios.TIOCM_RTS | termios.TIOCM_DTR
    before = array('i', [0])
    try:
        ioctl(fd, termios.TIOCMGET, before, True)
        previous = before[0]
    except OSError:
        previous = None
    ioctl(fd, termios.TIOCMBIC, array('i', [mask]))
    after = array('i', [0])
    ioctl(fd, termios.TIOCMGET, after, True)
    if after[0] & mask:
        raise OSError('驱动未释放RTS/DTR，停止连接；暂不按Reset。')
    return previous, after[0]


def send_command(fd, text):
    command = text.strip().upper()
    allowed = command in ('RUN!', 'STOP!', 'S!', 'F!', 'B!', 'L!', 'R!')
    if command.startswith('V') and command.endswith('!'):
        number = command[1:-1]
        allowed = number.isascii() and number.isdigit() and len(number) <= 3 and 40 <= int(number) <= 200
    if command.startswith('TIME') and command.endswith('!'):
        number = command[4:-1]
        allowed = number.isascii() and number.isdigit() and len(number) <= 2 and 1 <= int(number) <= 30
    if not allowed:
        raise ValueError('命令：RUN!/STOP!/F!/B!/L!/R!/S!、V40!..V200!、TIME1!..TIME30!；q退出。')
    payload = command.encode('ascii')
    pending = payload
    deadline = time.monotonic() + 0.2
    while pending:
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not select.select([], [fd], [], remaining)[1]:
            raise TimeoutError('命令写入超时；不要继续RUN，用使能OFF停车。')
        try:
            written = os.write(fd, pending)
        except BlockingIOError:
            continue
        if written == 0:
            raise OSError('串口写入失败')
        pending = pending[written:]
    return payload


def safe_display(data):
    text = data.decode('utf-8', errors='replace')
    return ''.join(c if c.isprintable() or c in '\r\n\t' else f'<{ord(c):02X}>'
                   for c in text)


def record(fd, output, seconds=None, commands=True, show=True):
    output.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now(ZONE).strftime('%Y-%m-%d-%H-%M-%S-%f')
    json_path = output / f'records-local-{stamp}.json'
    raw_path = output / f'records-local-{stamp}.bin'
    received = 0
    start = last_notice = time.monotonic()
    last_receive = start
    first = True
    with json_path.open('w', encoding='utf-8') as events, raw_path.open('wb') as raw:
        events.write('[\n')

        def save(kind, data):
            nonlocal first
            now = datetime.datetime.now(ZONE)
            event = {'type': kind, 'data': data.decode('utf-8', errors='replace'),
                     'timestamp': int(now.timestamp() * 1000),
                     'time': now.strftime('%Y-%m-%d %H:%M:%S:') + f'{now.microsecond // 1000:03d}',
                     'byteLength': len(data), 'display': 'ascii'}
            events.write(('' if first else ',\n') + json.dumps(event, ensure_ascii=False))
            events.flush()
            first = False

        print(f'记录：{json_path}', flush=True)
        print('115200 / 8N1。保持使能OFF；真实串口需看到控制线释放提示后再按Reset。', flush=True)
        if commands:
            print('输入F!/B!/L!/R!/STOP!或RUN!后回车发送；q回车或Ctrl+C退出保存。', flush=True)
        try:
            while seconds is None or time.monotonic() - start < seconds:
                sources = [fd]
                if commands:
                    sources.append(sys.stdin)
                wait = 0.25
                if seconds is not None:
                    wait = min(wait, max(0.0, seconds - (time.monotonic() - start)))
                ready, _, _ = select.select(sources, [], [], wait)
                if fd in ready:
                    try:
                        data = os.read(fd, 4096)
                    except BlockingIOError:
                        data = None
                    if data == b'':
                        print('\n串口断开，记录已保存。', flush=True)
                        break
                    if data:
                        raw.write(data)
                        raw.flush()
                        save('read', data)
                        received += len(data)
                        last_receive = time.monotonic()
                        if show:
                            print(safe_display(data), end='', flush=True)
                if commands and sys.stdin in ready:
                    line = sys.stdin.readline()
                    if not line or line.strip().lower() in ('q', 'quit', 'exit'):
                        break
                    if line.strip():
                        try:
                            payload = send_command(fd, line)
                            save('write', payload)
                            print(f'[发送] {payload.decode()}', flush=True)
                        except (OSError, ValueError, TimeoutError) as exc:
                            print(f'[未完成发送] {exc}', flush=True)
                now = time.monotonic()
                if now - last_receive >= 5 and now - last_notice >= 5:
                    print(f'[接收循环仍在运行，已{int(now - last_receive)}秒没有新数据；'
                          '可用q退出保存，不要发送RUN。]', flush=True)
                    last_notice = now
        except KeyboardInterrupt:
            print('\n结束接收。', flush=True)
        finally:
            events.write('\n]\n')
            events.flush()
    print(f'保存完成：{received}字节；{json_path}', flush=True)
    return json_path, raw_path


def self_test():
    import pty
    import tempfile
    import threading
    # Exercise ioctl sequencing and failure handling without opening USB.
    mask = termios.TIOCM_RTS | termios.TIOCM_DTR
    modem = {'bits': mask | termios.TIOCM_CTS, 'calls': []}

    def mock_ioctl(fd, request, value, *args):
        assert fd == -1
        modem['calls'].append(request)
        if request == termios.TIOCMGET:
            value[0] = modem['bits']
        elif request == termios.TIOCMBIC:
            assert value[0] == mask
            modem['bits'] &= ~value[0]
        else:
            raise AssertionError('unexpected ioctl')
        return 0

    before, after = release_boot_controls(-1, mock_ioctl)
    assert before & mask == mask and after == termios.TIOCM_CTS
    assert modem['calls'] == [termios.TIOCMGET, termios.TIOCMBIC, termios.TIOCMGET]

    def stuck_ioctl(fd, request, value, *args):
        if request == termios.TIOCMGET:
            value[0] = termios.TIOCM_RTS
        return 0

    try:
        release_boot_controls(-1, stuck_ioctl)
    except OSError:
        pass
    else:
        raise AssertionError('asserted RTS accepted')

    def unsupported_ioctl(*args):
        raise OSError(errno.ENOTTY, 'unsupported modem control')

    try:
        release_boot_controls(-1, unsupported_ioctl)
    except OSError:
        pass
    else:
        raise AssertionError('unsupported modem control accepted')
    master, slave = pty.openpty()
    previous = configure(slave)
    os.set_blocking(slave, False)
    os.set_blocking(master, False)
    chunks = [b'DPI T=0 L=0 R=0 ', b'\r\nboot\r\n',
              b'DPI T=0 L=0 R=0\r\n', b'DPI T=0 L=0 ',
              b'\r\nboot2\r\n', b'DPI T=0 L=0 R=0\r\n']
    expected = b''.join(chunks)

    def producer():
        for chunk in chunks:
            time.sleep(0.03)
            os.write(master, chunk)

    worker = threading.Thread(target=producer)
    worker.start()
    try:
        output = Path(tempfile.mkdtemp(prefix='serial-recorder-test-'))
        saved, raw = record(slave, output, seconds=0.5, commands=False, show=False)
        worker.join()
        events = json.loads(saved.read_text())
        assert raw.read_bytes() == expected
        assert ''.join(e['data'] for e in events).encode() == expected
        assert sum(e['byteLength'] for e in events) == len(expected)
        assert send_command(slave, 'STOP!\n') == b'STOP!'
        assert select.select([master], [], [], 0.2)[0]
        assert os.read(master, 20) == b'STOP!'
        for command in ('F!', 'B!', 'L!', 'R!', 'S!', 'V150!', 'TIME20!'):
            assert send_command(slave, command) == command.encode()
            assert select.select([master], [], [], 0.2)[0]
            assert os.read(master, 20) == command.encode()
        for command in ('V201!', 'V-80!', 'TIME0!', 'TIME31!'):
            try:
                send_command(slave, command)
            except ValueError:
                pass
            else:
                raise AssertionError('out-of-range manual command accepted')
        try:
            send_command(slave, 'AT+RST')
        except ValueError:
            pass
        else:
            raise AssertionError('unexpected command accepted')
        os.write(master, b'RUN!\r\n')
        time.sleep(0.02)
        try:
            assert os.read(master, 20) == b''
        except BlockingIOError:
            pass
        assert '\x1b' not in safe_display(b'\x1b[2J\x00test\r\n')
        print('PASS: modem RTS/DTR release mask/order/readback, stuck/unsupported ioctl rejected; repeated reset-like truncated streams preserved; valid JSON/raw bytes; bounded receive loop; STOP command; invalid command rejected; display escaping.')
    finally:
        termios.tcsetattr(slave, termios.TCSANOW, previous)
        os.close(slave)
        os.close(master)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port')
    parser.add_argument('--seconds', type=float)
    parser.add_argument('--no-commands', action='store_true')
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    ports = sorted(set(glob.glob('/dev/cu.usb*') + glob.glob('/dev/cu.wch*') + glob.glob('/dev/cu.SLAB*')))
    if args.port:
        port = args.port
    elif len(ports) == 1:
        port = ports[0]
    elif not ports:
        raise SystemExit('未找到USB串口，请插入串口线后再启动。')
    else:
        for index, candidate in enumerate(ports, 1):
            print(f'{index}. {candidate}')
        answer = input('选择串口编号：').strip()
        if not answer.isdigit() or not 1 <= int(answer) <= len(ports):
            raise SystemExit('编号无效，未连接。')
        port = ports[int(answer) - 1]
    lsof = Path('/usr/sbin/lsof')
    if lsof.exists():
        try:
            busy = subprocess.run([str(lsof), '-t', port], capture_output=True, text=True, timeout=2)
            if busy.stdout.strip():
                raise SystemExit('该串口仍被其他程序占用。先断开网页串口，或关闭串口助手标签页，再启动本工具。')
        except subprocess.TimeoutExpired:
            raise SystemExit('检查串口占用超时，未打开串口。请关闭串口助手标签页后重试。')
    print(f'连接：{port}；不会自动发送RUN，打开后会释放RTS/DTR以避免进入下载模式。', flush=True)
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    previous = None
    try:
        previous = configure(fd)
        if hasattr(termios, 'TIOCEXCL'):
            fcntl.ioctl(fd, termios.TIOCEXCL)
        before, after = release_boot_controls(fd)
        if before is not None:
            print(f'[控制线打开时] RTS置位={int(bool(before & termios.TIOCM_RTS))} '
                  f'DTR置位={int(bool(before & termios.TIOCM_DTR))}', flush=True)
        print('[控制线已释放] RTS未置位，DTR未置位（驱动读回；不是引脚电压测量）。', flush=True)
        print('保持使能OFF，现在按一次板上Reset，让程序重新启动；随后可再次Reset验证。', flush=True)
        record(fd, Path(__file__).resolve().parent / '接收记录', args.seconds, not args.no_commands)
    except OSError as exc:
        print(f'串口错误：{exc}。用使能OFF停车，保存的JSON/原始数据仍可分析。')
    finally:
        if previous is not None:
            try:
                termios.tcsetattr(fd, termios.TCSANOW, previous)
            except OSError:
                pass
        os.close(fd)


if __name__ == '__main__':
    main()
