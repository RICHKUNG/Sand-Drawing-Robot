"""Render the three drawing passes of the sand table from the firmware sources.

Reads the point arrays in firmware/sand_table/gcode.h and endcode.h and
re-evaluates the two polar functions used in runfunc(), so the preview always
matches what the robot will draw.

    python tools/plot_toolpaths.py            # writes docs/media/toolpaths.png
"""
import re
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
FW = ROOT / "firmware" / "sand_table"
RADIUS = 276.0  # sum of the two arm lengths (mm), same as setupsand.h


def load_points(header: Path) -> np.ndarray:
    text = header.read_text(encoding="utf-8")
    pts = re.findall(r"\{\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\}", text)
    # The firmware stores points with the table centre at (276, 276).
    return np.array([[float(x), float(y)] for x, y in pts]) - RADIUS


def polar_pass() -> np.ndarray:
    """Same loops as runfunc() in sand_table.ino."""
    pts, t, r = [], 30.0, 0.0
    while r < RADIUS:
        r = 1.3 * t + 20 * np.sin(5 * t)
        pts.append((r, t))
        t += 0.01
    while t > 0:
        r = 50 - 200 * np.cos(2 * t)
        pts.append((r, t))
        t -= 0.001
    p = np.array(pts)
    return np.c_[p[:, 0] * np.cos(p[:, 1]), p[:, 0] * np.sin(p[:, 1])]


def main() -> None:
    g, e, f = load_points(FW / "gcode.h"), load_points(FW / "endcode.h"), polar_pass()
    panels = [
        (g, f"1 · G-code pattern ({len(g):,} points)"),
        (f, "2 · Polar functions r(θ)"),
        (e, f"3 · Closing pattern ({len(e):,} points)"),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(12, 4.6), facecolor="white")
    for ax, (xy, title) in zip(axes, panels):
        ax.add_patch(plt.Circle((0, 0), RADIUS, color="#f3eee4", zorder=0))
        ax.add_patch(plt.Circle((0, 0), RADIUS, fill=False, color="#c9b48f", lw=3))
        ax.plot(xy[:, 0], xy[:, 1], color="#3a3226", lw=0.6)
        ax.set_xlim(-290, 290)
        ax.set_ylim(-290, 290)
        ax.set_aspect("equal")
        ax.axis("off")
        ax.set_title(title, fontsize=12, color="#333", pad=10)
    fig.subplots_adjust(top=0.88, bottom=0.02, left=0.01, right=0.99, wspace=0.05)
    out = ROOT / "docs" / "media" / "toolpaths.png"
    fig.savefig(out, dpi=110)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
