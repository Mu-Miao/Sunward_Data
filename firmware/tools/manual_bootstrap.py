#!/usr/bin/env python3
"""Hold C10B BOOT0 active and wait for a manual RESET press."""

import fcntl
import os
import select
import struct
import sys
import termios
import time


PORT = "/dev/cu.usbserial-5B310185631"
TIMEOUT_SECONDS = 45


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


def main() -> int:
    fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY)
    try:
        configure_uart(fd)

        # CH9102 DTR/RTS outputs are active-low. Assert both: RTS pulls
        # BOOT0 high through Q1, while DTR stays in the non-reset state.
        signals = termios.TIOCM_RTS | termios.TIOCM_DTR
        set_modem_bits(fd, termios.TIOCMBIS, signals)

        print("READY: BOOT0 is held active; press the board RESET button once.", flush=True)
        deadline = time.monotonic() + TIMEOUT_SECONDS

        while time.monotonic() < deadline:
            os.write(fd, b"\x7f")
            readable, _, _ = select.select([fd], [], [], 0.4)
            if readable:
                reply = os.read(fd, 64)
                if b"\x79" in reply:
                    # Release BOOT0 after the ROM bootloader has acknowledged.
                    set_modem_bits(fd, termios.TIOCMBIC, termios.TIOCM_RTS)
                    print("ACK: STM32 ROM bootloader detected.", flush=True)
                    return 0
            time.sleep(0.1)

        print("TIMEOUT: RESET was not detected.", file=sys.stderr, flush=True)
        return 1
    finally:
        os.close(fd)


if __name__ == "__main__":
    raise SystemExit(main())
