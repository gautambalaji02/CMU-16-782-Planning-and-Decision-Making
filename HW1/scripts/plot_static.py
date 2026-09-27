"""
Static (non-animated) plot of a solved run, for the write-up.

Usage (from the scripts directory, after running ./run_test mapN.txt):
    python plot_static.py ../maps/mapN.txt [../output/robot_trajectory.txt] [out.png]

Shows the costmap (obstacles in black), the full target trajectory, the
robot trajectory, the start positions, where the robot waited, and the
catch point.
"""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm

from visualizer import parse_mapfile, parse_robot_trajectory_file


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    map_file = sys.argv[1]
    traj_file = sys.argv[2] if len(sys.argv) > 2 else "../output/robot_trajectory.txt"
    out_file = sys.argv[3] if len(sys.argv) > 3 else "scripts/result.png"

    xs, ys, thresh, rx, ry, target, costmap = parse_mapfile(map_file)
    robot = parse_robot_trajectory_file(traj_file)

    free = np.ma.masked_where(costmap >= thresh, costmap)
    fig, ax = plt.subplots(figsize=(8, 8 * ys / max(xs, 1) if xs >= ys else 8))
    ax.set_facecolor("black")                       # obstacles
    vmin, vmax = max(1, free.min()), max(2, free.max())
    im = ax.imshow(free, cmap="viridis", norm=LogNorm(vmin=vmin, vmax=vmax),
                   origin="upper", interpolation="nearest")
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04, label="cell cost (log scale)")

    tx = [p["x"] - 1 for p in target]
    ty = [p["y"] - 1 for p in target]
    ax.plot(tx, ty, color="magenta", lw=1.5, label="target trajectory")
    ax.plot(tx[0], ty[0], "o", color="magenta", ms=8)

    px = [p["x"] - 1 for p in robot]
    py = [p["y"] - 1 for p in robot]
    ax.plot(px, py, color="red", lw=2, label="robot trajectory")
    ax.plot(px[0], py[0], "o", color="lime", ms=9, mec="k", label="robot start")

    best_wait, cur, start = (0, None), 0, 0
    for k in range(1, len(robot)):
        if (robot[k]["x"], robot[k]["y"]) == (robot[k - 1]["x"], robot[k - 1]["y"]):
            cur += robot[k]["t"] - robot[k - 1]["t"]
            if cur > best_wait[0]:
                best_wait = (cur, k)
        else:
            cur = 0
    if best_wait[0] > 0:
        k = best_wait[1]
        ax.plot(px[k], py[k], "s", color="orange", ms=10, mec="k",
                label=f"waits {best_wait[0]} steps")

    ax.plot(px[-1], py[-1], "*", color="yellow", ms=18, mec="k",
            label=f"catch at t={robot[-1]['t']}")
    ax.set_xlim(-0.5, xs - 0.5)
    ax.set_ylim(ys - 0.5, -0.5)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title(map_file.split("/")[-1])
    ax.legend(loc="best", fontsize=8, framealpha=0.85)
    fig.tight_layout()
    fig.savefig(out_file, dpi=130)
    print("saved", out_file)


if __name__ == "__main__":
    main()
