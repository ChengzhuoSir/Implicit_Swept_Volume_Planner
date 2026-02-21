#!/usr/bin/env python3
"""Generate a deterministic maze-like PGM map and YAML metadata.

Map size is 200x200 at 0.05m resolution (10m x 10m), matching office map scale.
"""

from pathlib import Path

W = 200
H = 200
FREE = 255
OCC = 0


def draw_hline(img, y, x0, x1):
    for x in range(max(0, x0), min(W, x1)):
        img[y][x] = OCC


def draw_vline(img, x, y0, y1):
    for y in range(max(0, y0), min(H, y1)):
        img[y][x] = OCC


def carve_v_doors(img, x, doors):
    for y0, y1 in doors:
        for y in range(max(0, y0), min(H, y1)):
            img[y][x] = FREE


def carve_h_doors(img, y, doors):
    for x0, x1 in doors:
        for x in range(max(0, x0), min(W, x1)):
            img[y][x] = FREE


def main():
    out_dir = Path(__file__).resolve().parents[1] / "maps"
    out_dir.mkdir(parents=True, exist_ok=True)
    pgm_path = out_dir / "maze.pgm"
    yaml_path = out_dir / "maze.yaml"

    img = [[FREE for _ in range(W)] for _ in range(H)]

    # Border walls.
    for x in range(W):
        img[0][x] = OCC
        img[H - 1][x] = OCC
    for y in range(H):
        img[y][0] = OCC
        img[y][W - 1] = OCC

    # Vertical maze walls with door gaps.
    v_walls = [40, 80, 120, 160]
    for x in v_walls:
        draw_vline(img, x, 1, H - 1)
    carve_v_doors(img, 40, [(20, 38), (118, 142)])
    carve_v_doors(img, 80, [(58, 82), (158, 182)])
    carve_v_doors(img, 120, [(28, 52), (98, 118)])
    carve_v_doors(img, 160, [(38, 58), (138, 172)])

    # Horizontal maze walls with door gaps.
    h_walls = [60, 110, 160]
    for y in h_walls:
        draw_hline(img, y, 1, W - 1)
    carve_h_doors(img, 60, [(22, 46), (128, 152)])
    carve_h_doors(img, 110, [(68, 92), (168, 192)])
    carve_h_doors(img, 160, [(38, 62), (98, 122)])

    # A few block obstacles.
    blocks = [
        (20, 20, 32, 36),
        (55, 140, 72, 158),
        (95, 30, 112, 48),
        (145, 85, 165, 105),
    ]
    for x0, y0, x1, y1 in blocks:
        for y in range(y0, y1):
            for x in range(x0, x1):
                if 0 <= x < W and 0 <= y < H:
                    img[y][x] = OCC

    # Guaranteed traversable corridor for 0.6m-class robots.
    # This keeps the map maze-like while ensuring start-goal connectivity.
    centerline = [(20, 20), (20, 85), (75, 85), (75, 135), (135, 135), (135, 175), (175, 175)]
    half_w = 12
    for i in range(1, len(centerline)):
        x0, y0 = centerline[i - 1]
        x1, y1 = centerline[i]
        if x0 == x1:
            ys, ye = sorted((y0, y1))
            for y in range(ys, ye + 1):
                for x in range(x0 - half_w, x0 + half_w + 1):
                    if 0 <= x < W and 0 <= y < H:
                        img[y][x] = FREE
        elif y0 == y1:
            xs, xe = sorted((x0, x1))
            for x in range(xs, xe + 1):
                for y in range(y0 - half_w, y0 + half_w + 1):
                    if 0 <= x < W and 0 <= y < H:
                        img[y][x] = FREE

    # Ensure start/goal neighborhoods are free.
    for y in range(8, 30):
        for x in range(8, 30):
            img[y][x] = FREE
    for y in range(150, 192):
        for x in range(150, 192):
            img[y][x] = FREE

    # Write PGM (P5 binary).
    with pgm_path.open("wb") as f:
        header = f"P5\n{W} {H}\n255\n".encode("ascii")
        f.write(header)
        # PGM stores top-to-bottom rows.
        for y in range(H):
            f.write(bytes(img[y]))

    yaml_text = "\n".join(
        [
            "image: maze.pgm",
            "resolution: 0.05",
            "origin: [0.0, 0.0, 0.0]",
            "negate: 0",
            "occupied_thresh: 0.65",
            "free_thresh: 0.196",
            "",
        ]
    )
    yaml_path.write_text(yaml_text, encoding="utf-8")

    print(f"Generated: {pgm_path}")
    print(f"Generated: {yaml_path}")


if __name__ == "__main__":
    main()
