#!/usr/bin/env python3
"""Generate the static occupancy map paired with research_warehouse.sdf."""

from pathlib import Path
import sys


RESOLUTION = 0.05
ORIGIN_X = -6.0
ORIGIN_Y = -4.0
WIDTH = 240
HEIGHT = 160

# (center_x, center_y, size_x, size_y), identical to the world collision boxes.
OBSTACLES = (
    (0.0, 3.95, 12.0, 0.10),
    (0.0, -3.95, 12.0, 0.10),
    (-5.95, 0.0, 0.10, 8.0),
    (5.95, 0.0, 0.10, 8.0),
    (-1.0, 1.4, 0.60, 3.60),
    (2.3, -1.2, 3.20, 0.60),
    (4.3, 1.9, 0.60, 2.80),
    (-3.6, -1.8, 1.00, 1.00),
)


def occupied(x: float, y: float) -> bool:
    return any(
        abs(x - center_x) <= 0.5 * size_x
        and abs(y - center_y) <= 0.5 * size_y
        for center_x, center_y, size_x, size_y in OBSTACLES
    )


def generate(output: Path) -> None:
    pixels = bytearray()
    for image_row in range(HEIGHT):
        map_row = HEIGHT - 1 - image_row
        y = ORIGIN_Y + (map_row + 0.5) * RESOLUTION
        for column in range(WIDTH):
            x = ORIGIN_X + (column + 0.5) * RESOLUTION
            pixels.append(0 if occupied(x, y) else 254)

    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("wb") as stream:
        stream.write(f"P5\n{WIDTH} {HEIGHT}\n255\n".encode("ascii"))
        stream.write(pixels)


if __name__ == "__main__":
    default_output = Path(__file__).resolve().parents[1] / "maps" / "research_warehouse.pgm"
    generate(Path(sys.argv[1]) if len(sys.argv) > 1 else default_output)
