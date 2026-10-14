#!/usr/bin/env python3
"""Convert an animated GIF to SSD1306 128x64 page-format C arrays."""

from __future__ import annotations

import argparse
import re
from pathlib import Path

from PIL import Image, ImageOps


WIDTH = 128
HEIGHT = 64


def c_identifier(value: str) -> str:
    value = re.sub(r"[^A-Za-z0-9_]", "_", value)
    if not value or value[0].isdigit():
        value = "anim_" + value
    return value


def prepare_frame(
    frame: Image.Image,
    threshold: int,
    invert: bool,
    background: str,
    use_alpha: bool,
) -> Image.Image:
    source = frame.convert("RGBA")
    if use_alpha:
        grayscale = source.getchannel("A")
    else:
        background_value = 255 if background == "white" else 0
        flattened = Image.new(
            "RGBA", source.size,
            (background_value, background_value, background_value, 255),
        )
        flattened.alpha_composite(source)
        grayscale = flattened.convert("L")

    fitted = ImageOps.contain(grayscale, (WIDTH, HEIGHT), Image.Resampling.LANCZOS)
    canvas = Image.new("L", (WIDTH, HEIGHT), 0)
    canvas.paste(fitted, ((WIDTH - fitted.width) // 2, (HEIGHT - fitted.height) // 2))

    if invert:
        canvas = ImageOps.invert(canvas)
    return canvas.point(lambda pixel: 255 if pixel >= threshold else 0, mode="1")


def pack_ssd1306(image: Image.Image) -> list[int]:
    pixels = image.load()
    packed: list[int] = []
    for page in range(HEIGHT // 8):
        for x in range(WIDTH):
            value = 0
            for bit in range(8):
                if pixels[x, page * 8 + bit]:
                    value |= 1 << bit
            packed.append(value)
    return packed


def format_bytes(data: list[int]) -> str:
    lines = []
    for offset in range(0, len(data), 16):
        chunk = data[offset : offset + 16]
        lines.append("        " + ", ".join(f"0x{value:02x}" for value in chunk))
    return ",\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path, help="input animated GIF")
    parser.add_argument("output", type=Path, help="output C header")
    parser.add_argument("--name", default="oled_animation", help="C array prefix")
    parser.add_argument("--threshold", type=int, default=128, choices=range(1, 256))
    parser.add_argument("--invert", action="store_true")
    parser.add_argument("--background", choices=("black", "white"), default="black",
                        help="color used behind transparent pixels")
    parser.add_argument("--alpha", action="store_true",
                        help="use GIF opacity as the image; useful for stickers")
    parser.add_argument("--max-frames", type=int, default=0,
                        help="limit exported frames; zero exports all")
    args = parser.parse_args()

    name = c_identifier(args.name)
    frames: list[list[int]] = []
    delays: list[int] = []

    with Image.open(args.input) as gif:
        frame_count = getattr(gif, "n_frames", 1)
        if args.max_frames > 0:
            frame_count = min(frame_count, args.max_frames)

        for index in range(frame_count):
            gif.seek(index)
            duration = max(10, int(gif.info.get("duration", 100)))
            converted = prepare_frame(
                gif, args.threshold, args.invert, args.background, args.alpha
            )
            frames.append(pack_ssd1306(converted))
            delays.append(duration)

    guard = c_identifier(args.output.stem).upper() + "_H"
    parts = [
        f"#ifndef {guard}",
        f"#define {guard}",
        "",
        "#include <stdint.h>",
        "",
        f"#define {name.upper()}_FRAME_COUNT {len(frames)}U",
        "",
        f"static const uint16_t {name}_delays_ms[{len(frames)}] = {{",
        "    " + ", ".join(f"{delay}U" for delay in delays),
        "};",
        "",
        f"static const uint8_t {name}_frames[{len(frames)}][1024] = {{",
    ]

    for index, frame in enumerate(frames):
        parts.append(f"    /* Frame {index} */")
        parts.append("    {")
        parts.append(format_bytes(frame))
        parts.append("    },")

    parts.extend(["};", "", f"#endif /* {guard} */", ""])
    args.output.write_text("\n".join(parts), encoding="utf-8")

    kib = len(frames) * 1024 / 1024
    print(f"Wrote {len(frames)} frames ({kib:.0f} KiB of pixel data) to {args.output}")


if __name__ == "__main__":
    main()
