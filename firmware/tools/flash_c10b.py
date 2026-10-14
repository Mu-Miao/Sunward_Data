#!/usr/bin/env python3
"""Flash the C10B's STM32F103RCT6 through its CH9102 USB-UART bridge."""

from __future__ import annotations

import fcntl
import os
from pathlib import Path
import select
import struct
import sys
import termios
import time


PORT = "/dev/cu.usbserial-5B310185631"
IMAGE = Path(__file__).resolve().parent.parent / "build" / "c10b_motor_test.bin"
FLASH_BASE = 0x08000000
EXPECTED_DEVICE_ID = 0x0414  # STM32F10x high-density
PAGE_SIZE = 2048
ACK = 0x79
NACK = 0x1F


class FlashError(RuntimeError):
    pass


def set_modem_bits(fd: int, request: int, bits: int) -> None:
    fcntl.ioctl(fd, request, struct.pack("I", bits))


def configure_uart(fd: int) -> None:
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] &= ~(termios.CSIZE | termios.CSTOPB | termios.PARODD)
    attrs[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD | termios.PARENB
    attrs[3] = 0
    attrs[4] = termios.B57600
    attrs[5] = termios.B57600
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 1
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)


def read_exact(fd: int, length: int, timeout: float, context: str) -> bytes:
    deadline = time.monotonic() + timeout
    result = bytearray()
    while len(result) < length:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise FlashError(f"timeout while {context}")
        readable, _, _ = select.select([fd], [], [], remaining)
        if readable:
            result.extend(os.read(fd, length - len(result)))
    return bytes(result)


def wait_ack(fd: int, context: str, timeout: float = 3.0) -> None:
    reply = read_exact(fd, 1, timeout, context)[0]
    if reply == NACK:
        raise FlashError(f"device returned NACK while {context}")
    if reply != ACK:
        raise FlashError(f"unexpected byte 0x{reply:02x} while {context}")


def command(fd: int, value: int, context: str) -> None:
    os.write(fd, bytes((value, value ^ 0xFF)))
    wait_ack(fd, context)


def address_packet(address: int) -> bytes:
    raw = address.to_bytes(4, "big")
    checksum = raw[0] ^ raw[1] ^ raw[2] ^ raw[3]
    return raw + bytes((checksum,))


def synchronize(fd: int) -> None:
    # The board inverts the active-low CH9102 control outputs. Asserting both
    # holds BOOT0 active without holding NRST low; the user supplies reset.
    signals = termios.TIOCM_RTS | termios.TIOCM_DTR
    set_modem_bits(fd, termios.TIOCMBIS, signals)
    print("READY: press the blue RESET button once now.", flush=True)
    deadline = time.monotonic() + 45
    while time.monotonic() < deadline:
        os.write(fd, b"\x7f")
        readable, _, _ = select.select([fd], [], [], 0.4)
        if readable:
            reply = os.read(fd, 64)
            if bytes((ACK,)) in reply:
                # BOOT0 can be released once the ROM bootloader is running.
                set_modem_bits(fd, termios.TIOCMBIC, termios.TIOCM_RTS)
                print("ACK: ROM bootloader connected.", flush=True)
                return
        time.sleep(0.1)
    raise FlashError("RESET was not detected within 45 seconds")


def get_commands(fd: int) -> tuple[int, set[int]]:
    command(fd, 0x00, "requesting bootloader command list")
    count_minus_one = read_exact(fd, 1, 2, "reading command-list length")[0]
    payload = read_exact(fd, count_minus_one + 1, 2, "reading command list")
    wait_ack(fd, "finishing command-list request")
    return payload[0], set(payload[1:])


def get_device_id(fd: int) -> int:
    command(fd, 0x02, "requesting device ID")
    count_minus_one = read_exact(fd, 1, 2, "reading device-ID length")[0]
    raw = read_exact(fd, count_minus_one + 1, 2, "reading device ID")
    wait_ack(fd, "finishing device-ID request")
    return int.from_bytes(raw, "big")


def erase_pages(fd: int, supported: set[int], page_count: int) -> None:
    pages = list(range(page_count))
    if 0x44 in supported:
        command(fd, 0x44, "starting extended erase")
        body = bytearray((page_count - 1).to_bytes(2, "big"))
        for page in pages:
            body.extend(page.to_bytes(2, "big"))
    elif 0x43 in supported and page_count <= 256:
        command(fd, 0x43, "starting erase")
        body = bytearray((page_count - 1, *pages))
    else:
        raise FlashError("bootloader does not provide a compatible erase command")
    checksum = 0
    for value in body:
        checksum ^= value
    os.write(fd, body + bytes((checksum,)))
    wait_ack(fd, "erasing flash pages", timeout=15)


def write_memory(fd: int, address: int, data: bytes) -> None:
    command(fd, 0x31, f"starting write at 0x{address:08x}")
    os.write(fd, address_packet(address))
    wait_ack(fd, f"accepting write address 0x{address:08x}")
    length_minus_one = len(data) - 1
    checksum = length_minus_one
    for value in data:
        checksum ^= value
    os.write(fd, bytes((length_minus_one,)) + data + bytes((checksum,)))
    wait_ack(fd, f"writing 0x{address:08x}", timeout=5)


def read_memory(fd: int, address: int, length: int) -> bytes:
    command(fd, 0x11, f"starting verify at 0x{address:08x}")
    os.write(fd, address_packet(address))
    wait_ack(fd, f"accepting verify address 0x{address:08x}")
    length_minus_one = length - 1
    os.write(fd, bytes((length_minus_one, length_minus_one ^ 0xFF)))
    wait_ack(fd, f"accepting verify length at 0x{address:08x}")
    return read_exact(fd, length, 5, f"reading flash at 0x{address:08x}")


def start_application(fd: int) -> None:
    command(fd, 0x21, "starting application")
    os.write(fd, address_packet(FLASH_BASE))
    wait_ack(fd, "jumping to application")


def main() -> int:
    if not IMAGE.is_file():
        print(f"ERROR: firmware image not found: {IMAGE}", file=sys.stderr)
        return 1
    image = IMAGE.read_bytes()
    if not image:
        print("ERROR: firmware image is empty", file=sys.stderr)
        return 1
    padded = image + b"\xff" * ((-len(image)) % 4)
    page_count = (len(padded) + PAGE_SIZE - 1) // PAGE_SIZE

    fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY)
    try:
        configure_uart(fd)
        synchronize(fd)
        version, supported = get_commands(fd)
        device_id = get_device_id(fd)
        print(f"Bootloader v{version >> 4}.{version & 0x0f}; device ID 0x{device_id:04x}", flush=True)
        if device_id != EXPECTED_DEVICE_ID:
            raise FlashError(
                f"refusing to flash unexpected device 0x{device_id:04x}; "
                f"expected 0x{EXPECTED_DEVICE_ID:04x}"
            )
        required = {0x11, 0x21, 0x31}
        if not required.issubset(supported):
            missing = ", ".join(f"0x{x:02x}" for x in sorted(required - supported))
            raise FlashError(f"bootloader is missing required commands: {missing}")

        print(f"Erasing {page_count} flash page(s)...", flush=True)
        erase_pages(fd, supported, page_count)

        print(f"Writing {len(image)} bytes...", flush=True)
        for offset in range(0, len(padded), 256):
            chunk = padded[offset : offset + 256]
            write_memory(fd, FLASH_BASE + offset, chunk)
            print(f"  wrote {min(offset + len(chunk), len(image))}/{len(image)} bytes", flush=True)

        print("Verifying...", flush=True)
        readback = bytearray()
        for offset in range(0, len(padded), 256):
            length = min(256, len(padded) - offset)
            readback.extend(read_memory(fd, FLASH_BASE + offset, length))
        if bytes(readback) != padded:
            raise FlashError("verification mismatch; application was not started")

        print("VERIFY OK: firmware matches flash.", flush=True)
        start_application(fd)
        print("DONE: test firmware started; keep motor enable OFF for now.", flush=True)
        return 0
    except (FlashError, OSError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        os.close(fd)


if __name__ == "__main__":
    raise SystemExit(main())
