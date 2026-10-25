#!/usr/bin/env python3
"""Read running-car RAM over SWD without resetting, halting, or resuming it.

Run with the existing pyOCD environment, not system Python. Stop via Ctrl-C or
the output directory's STOP file. Never share the probe with a VS Code session.
"""
import argparse
from contextlib import contextmanager
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import signal
import struct
import time

from elftools.elf.elffile import ELFFile
from elftools.dwarf.dwarf_expr import DWARFExprParser


class Layout:
    def __init__(self, path):
        self.path = Path(path)
        self.handle = self.path.open('rb')
        self.elf = ELFFile(self.handle)
        if not self.elf.little_endian or self.elf.elfclass != 32:
            raise ValueError('Expected little-endian 32-bit firmware ELF')
        dwarf = self.elf.get_dwarf_info()
        found = []
        for cu in dwarf.iter_CUs():
            for die in cu.iter_DIEs():
                a = die.attributes
                if (die.tag == 'DW_TAG_variable' and 'DW_AT_name' in a
                        and a['DW_AT_name'].value == b'instance' and 'DW_AT_location' in a):
                    expr = DWARFExprParser(dwarf.structs).parse_expr(a['DW_AT_location'].value)
                    if len(expr) == 1 and expr[0].op_name == 'DW_OP_addr':
                        found.append((expr[0].args[0], die.get_DIE_from_attribute('DW_AT_type')))
        if len(found) != 1:
            raise ValueError(f'Expected one addressable instance, found {len(found)}')
        self.address, self.root = found[0]
        self.size = self.sizeof(self.root)
        if not (0x20000000 <= self.address and self.address + self.size <= 0x2000C000):
            raise ValueError('instance is outside STM32F103RC SRAM')

    @staticmethod
    def resolve(die):
        while die.tag in ('DW_TAG_typedef', 'DW_TAG_const_type', 'DW_TAG_volatile_type',
                          'DW_TAG_restrict_type'):
            die = die.get_DIE_from_attribute('DW_AT_type')
        return die

    def sizeof(self, die):
        die = self.resolve(die)
        if die.tag == 'DW_TAG_array_type':
            return self.sizeof(die.get_DIE_from_attribute('DW_AT_type')) * self.count(die)
        return die.attributes['DW_AT_byte_size'].value

    @staticmethod
    def count(die):
        result = 1
        for child in die.iter_children():
            if child.tag == 'DW_TAG_subrange_type':
                a = child.attributes
                result *= (a['DW_AT_count'].value if 'DW_AT_count' in a
                           else a['DW_AT_upper_bound'].value + 1)
        return result

    def decode(self, blob, die=None, offset=0):
        die = self.resolve(self.root if die is None else die)
        if die.tag == 'DW_TAG_structure_type':
            return {c.attributes['DW_AT_name'].value.decode(): self.decode(
                blob, c.get_DIE_from_attribute('DW_AT_type'),
                offset + c.attributes['DW_AT_data_member_location'].value)
                for c in die.iter_children() if c.tag == 'DW_TAG_member'}
        if die.tag == 'DW_TAG_array_type':
            element = die.get_DIE_from_attribute('DW_AT_type')
            stride = self.sizeof(element)
            return [self.decode(blob, element, offset + n * stride) for n in range(self.count(die))]
        size = self.sizeof(die)
        data = blob[offset:offset + size]
        encoding = die.attributes.get('DW_AT_encoding')
        if encoding and encoding.value == 4:
            value = struct.unpack('<f' if size == 4 else '<d', data)[0]
            return value if math.isfinite(value) else None
        return int.from_bytes(data, 'little', signed=bool(encoding and encoding.value in (5, 6)))

    def flash_segments(self):
        return [(int(s['p_paddr']), s.data()) for s in self.elf.iter_segments()
                if s['p_type'] == 'PT_LOAD' and s['p_filesz']
                and 0x08000000 <= s['p_paddr'] < 0x08040000]


STATES = ['IDLE', 'RUNNING', 'DONE', 'OFF', 'LOW', 'ADC_BAD', 'GAP', 'REVERSE',
          'FAST', 'STALL', 'UART_BAD', 'STOPPED', 'YAW_BAD', 'BRAKING', 'TIMEOUT',
          'SETTLE_BAD', 'LINE_LOST', 'LINE_BAD']
PHASES = ['IDLE', 'DIAGONAL', 'VERTEX_BRAKE', 'ALIGN', 'CAPTURE', 'ARC', 'FINAL_BRAKE',
          'DONE', 'FAULT', 'START_ALIGN', 'START_SETTLE', 'ALIGN_SETTLE', 'VERTEX_SEEK', 'ARC_ENTRY']


def select(d, names):
    return {k: d[k] for k in names.split()}


def summarize(d):
    r, t, rt = d['route'], d['route']['task3'], d['runtime']
    return {
        'runtime': select(rt, 'state old_tick old_ms dt_us fault_wheel bluetooth_on'),
        'state_name': STATES[rt['state']] if rt['state'] < len(STATES) else 'UNKNOWN',
        'heading': select(d['heading'], 'phase angle reference error gyro_angle bias bias_rate fusion_active fusion_learning wheel_rate samples io_us'),
        'line': select(d['line'], 'raw valid count error sample_ms'),
        'route': select(r, 'motion distance_mm targets base elapsed_ms start_ms'),
        'task3': select(t, 'phase fault leg start_heading_deg leg_start_mm phase_start_mm phase_ms lost_seen loss_seek gap_seek loss_limits loss_duration_ms loss_distance_mm loss_turn_deg') |
                 {k: t[k] for k in ('entry_yaw_deg', 'entry_diff_mmps', 'edge_follow_active', 'resume_start_mmps') if k in t},
        'phase_name': PHASES[t['phase']] if t['phase'] < len(PHASES) else 'UNKNOWN',
        'task4': r['task4'],
        'wheel': [select(w, 'velocity command integral bad_sign_us') for w in d['wheel']['items']],
        'power': d['power'],
        'button': select(d['button'], 'raw stable armed action selected_motion events'),
        'reverse_trace': r['reverse_trace'], 'edge_trace': r['edge_trace'],
        'vertex_trace': r['vertex_trace'], 'line_history': r['task3_trace'],
    }


def atomic_json(path, value):
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False) + '\n')
    tmp.replace(path)


@contextmanager
def capture_status(output):
    """Mark a dead USB connection as failed instead of leaving 'connected'."""
    status_path = output.parent / 'latest_session.json'
    try:
        yield
    except Exception as error:
        atomic_json(status_path, {'directory': str(output.resolve()), 'status': 'disconnected',
                                 'error': str(error), 'ended_at': datetime.now().astimezone().isoformat()})
        raise
    else:
        atomic_json(status_path, {'directory': str(output.resolve()), 'status': 'stopped',
                                 'ended_at': datetime.now().astimezone().isoformat()})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--elf', type=Path, default=Path('build/Debug/Sunward.elf'))
    parser.add_argument('--probe', default='ATK-D62E11F3')
    parser.add_argument('--target', default='stm32f103rc')
    parser.add_argument('--hz', type=float, default=20)
    parser.add_argument('--seconds', type=float, default=0, help='0: until STOP or Ctrl-C')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--layout-only', action='store_true')
    args = parser.parse_args()
    if not (0 < args.hz <= 50) or args.seconds < 0:
        parser.error('Require 0 < hz <= 50 and seconds >= 0')
    layout = Layout(args.elf)
    if args.layout_only:
        print(json.dumps({'address': hex(layout.address), 'size': layout.size,
                          'zero_decode': summarize(layout.decode(bytes(layout.size)))}, indent=2))
        return

    from pyocd.core.helpers import ConnectHelper
    from pyocd.core.target import Target

    output = args.output or Path('build/pyocd_capture') / datetime.now().strftime('%Y%m%d_%H%M%S')
    output.mkdir(parents=True, exist_ok=False)
    atomic_json(output.parent / 'latest_session.json', {'directory': str(output.resolve()), 'status': 'connecting'})
    stop = False

    def request_stop(signum, frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    session = ConnectHelper.session_with_chosen_probe(
        blocking=False, unique_id=args.probe, options={
            'target_override': args.target, 'frequency': 4000000,
            'connect_mode': 'attach', 'resume_on_disconnect': False,
            'no_config': True, 'cache.enable_memory': False,
            'cache.enable_register': False, 'auto_unlock': False,
        })
    if session is None:
        raise RuntimeError('Probe not available; release the VS Code debug session first')
    # No halt/reset/resume/flash/core-register-write operations are used here.
    with capture_status(output), session:
        target = session.target
        before = target.get_state()
        verified = 0
        for address, expected in layout.flash_segments():
            for offset in range(0, len(expected), 1024):
                chunk = expected[offset:offset + 1024]
                actual = bytes(target.read_memory_block8(address + offset, len(chunk)))
                if actual != chunk:
                    raise RuntimeError(f'Board flash does not match ELF at {address + offset:#x}; refusing to decode RAM')
                verified += len(chunk)
        if not verified:
            raise RuntimeError('ELF has no flash data to verify')
        meta = {'elf': str(args.elf.resolve()), 'elf_sha256': hashlib.sha256(args.elf.read_bytes()).hexdigest(),
                'probe': args.probe, 'target': args.target, 'flash_bytes_verified': verified,
                'instance_address': hex(layout.address), 'instance_bytes': layout.size,
                'initial_core_state': before.name, 'requested_hz': args.hz,
                'sampling': 'asynchronous live RAM; not an atomic firmware snapshot',
                'started_at': datetime.now().astimezone().isoformat()}
        atomic_json(output / 'metadata.json', meta)
        atomic_json(output.parent / 'latest_session.json', {'directory': str(output.resolve()), 'status': 'connected', **meta})
        print('CONNECTED ' + json.dumps(meta), flush=True)
        if before == Target.State.HALTED:
            print('CORE_HALTED: not resuming automatically. Finish/release the previous debug stop before running.', flush=True)
        start = deadline = time.monotonic()
        count = 0
        last_key = None
        with (output / 'samples.jsonl').open('a', buffering=1) as log:
            while not stop and not (output / 'STOP').exists():
                if args.seconds and time.monotonic() - start >= args.seconds:
                    break
                begun = time.monotonic()
                state = target.get_state().name
                d = layout.decode(bytes(target.read_memory_block8(layout.address, layout.size)))
                record = {'host_time': datetime.now().astimezone().isoformat(),
                          'host_monotonic_s': begun, 'core_state': state,
                          **summarize(d), 'read_ms': (time.monotonic() - begun) * 1000,
                          'sample': count}
                log.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + '\n')
                atomic_json(output / 'latest.json', record)
                key = (record['core_state'], record['runtime']['state'], record['task3']['phase'],
                       record['task4']['completed_laps'], record['task3']['leg'])
                if key != last_key or count % max(1, int(args.hz * 10)) == 0:
                    h = record['heading']
                    print(f"SAMPLE {count} {state} {record['state_name']} {record['phase_name']} "
                          f"lap={record['task4']['completed_laps']} yaw={h['angle']} goal={h['reference']} "
                          f"read_ms={record['read_ms']:.1f}", flush=True)
                last_key = key
                count += 1
                deadline = max(deadline + 1 / args.hz, time.monotonic())
                time.sleep(max(0, deadline - time.monotonic()))
        atomic_json(output / 'ended.json', {'samples': count, 'ended_at': datetime.now().astimezone().isoformat()})
        print(f'STOPPED samples={count} directory={output}', flush=True)


if __name__ == '__main__':
    main()
