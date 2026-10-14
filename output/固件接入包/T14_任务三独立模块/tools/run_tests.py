from pathlib import Path
import argparse, subprocess, tempfile
parser = argparse.ArgumentParser()
parser.add_argument("--module-root", type=Path, default=Path(__file__).resolve().parents[2]/"T13_模块化重构")
args = parser.parse_args()

root = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='car_task3_') as tmp:
    for name, extra in [('host', []), ('sanitized', ['-fsanitize=address,undefined', '-fno-omit-frame-pointer'])]:
        binary = str(Path(tmp) / name)
        command = ['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-I'+str(root/'Core/Inc'),
                   str(root/'Core/Src/car_task3.c'), str(root/'tools/test_task3.c'), '-lm', *extra, '-o', binary]
        subprocess.run(command, check=True)
        result = subprocess.run([binary], check=True, capture_output=True, text=True)
        print(name+': '+result.stdout.strip())

    module = args.module_root.resolve()
    for name, extra in [('port', []), ('port_sanitized', ['-fsanitize=address,undefined', '-fno-omit-frame-pointer'])]:
        binary = str(Path(tmp) / name)
        command = ['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                   '-I'+str(root/'Core/Inc'), '-I'+str(module/'tools/tests'), '-I'+str(module/'Core/Inc'),
                   '-I/Users/mzh/STM32Cube/Sunward/Core/Inc',
                   str(root/'Core/Src/car_task3_port.c'), str(root/'tools/test_port.c'), '-lm', *extra, '-o', binary]
        subprocess.run(command, check=True)
        result = subprocess.run([binary], check=True, capture_output=True, text=True)
        print(name+': '+result.stdout.strip())
