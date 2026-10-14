#!/usr/bin/env python3
"""Build separate production modules with mock HAL; never access hardware."""
from pathlib import Path
import argparse
import os
import subprocess
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--project', type=Path, default=Path(__file__).resolve().parents[2])
parser.add_argument('--sanitize', action='store_true', help='Enable ASan and UBSan')
args = parser.parse_args()
root = args.project.resolve()
tests = Path(__file__).resolve().parent
modules = sorted((root / 'Core/Src').glob('car_*.c'))
assert len(modules) == 14, 'Expected all 14 car modules in Core/Src'
assert not any('#include' in line and '.c"' in line
               for path in modules for line in path.read_text().splitlines())
flags = [os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra', '-Werror',
         '-DCAR_APP_TEST', '-I' + str(tests), '-I' + str(root / 'Core/Inc')]
if args.sanitize:
    flags += ['-fsanitize=address,undefined', '-fno-omit-frame-pointer']
with tempfile.TemporaryDirectory(prefix='sunward-host-tests-') as folder:
    for suite in ['motion', 'task2', 'task3', 'task4', 'seek']:
        for public in [False, True]:
            name = suite + ('_car_app' if public else '_legacy_api')
            binary = str(Path(folder) / name)
            sources = [str(tests / ('test_' + suite + '.c'))] + [str(p) for p in modules]
            aliases = []
            if public:
                aliases = ['-DDualPITest_' + n + '=CarApp_' + n
                           for n in ['Begin', 'EnableBluetooth', 'EnableHeading', 'Poll']]
            else:
                sources.append(str(root / 'Core/Src/dual_pi_test.c'))
            subprocess.run(flags + aliases + sources + ['-lm', '-o', binary], check=True)
            result = subprocess.run([binary], capture_output=True, text=True)
            if result.returncode:
                print(result.stdout + result.stderr, flush=True)
                result.check_returncode()
            print(name + ': ' + result.stdout.strip())
print('PASS: motion/task2/task3/task4/seek regressions; CarApp and compatibility APIs; separate module linkage.')
