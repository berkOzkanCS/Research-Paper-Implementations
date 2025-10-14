import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import splprep, splev
import sys

def plot_smoothed_path(pathNodes, obstacles, start=None, goal=None, MAX_X=100, MAX_Y=100):
    if not pathNodes:
        print("No path nodes to plot.")
        return

    xs, ys = zip(*pathNodes)
    
    # Interpolate with splines
    tck, u = splprep([xs, ys], s=0)  # s=0 to pass through all points
    u_fine = np.linspace(0, 1, len(xs)*10)
    xs_smooth, ys_smooth = splev(u_fine, tck)
    
    fig, ax = plt.subplots(figsize=(8,8))
    ax.set_xlim(0, MAX_X)
    ax.set_ylim(0, MAX_Y)
    ax.set_aspect('equal')
    
    # Draw obstacles
    for cx, cy, r in obstacles:
        ax.add_patch(plt.Circle((cx, cy), r, color='black', fill=True))
    
    # Smoothed path
    ax.plot(xs_smooth, ys_smooth, color='blue', linewidth=2)
    ax.scatter(xs, ys, color='red', s=20)  # original points
    
    if start:
        ax.scatter(start[0], start[1], color='green', s=60, label='Start')
    if goal:
        ax.scatter(goal[0], goal[1], color='orange', s=60, label='Goal')
    
    ax.legend()
    plt.show()

def read_path_file(filename="path_data.txt"):
    MAX_X = MAX_Y = 0
    start = goal = None
    pathNodes = []
    obstacles = []

    with open(filename, "r") as f:
        lines = [line.strip() for line in f.readlines() if line.strip()]

    i = 0
    MAX_X, MAX_Y = map(int, lines[i].split())
    i += 1

    while i < len(lines):
        if lines[i].startswith("START"):
            _, x, y = lines[i].split()
            start = (float(x), float(y))
        elif lines[i].startswith("GOAL"):
            _, x, y = lines[i].split()
            goal = (float(x), float(y))
        elif lines[i] == "PATH":
            i += 1
            while i < len(lines) and lines[i] != "OBSTACLES":
                x, y = map(float, lines[i].split())
                pathNodes.append((x, y))
                i += 1
            continue
        elif lines[i] == "OBSTACLES":
            i += 1
            while i < len(lines):
                cx, cy, r = map(float, lines[i].split())
                obstacles.append((cx, cy, r))
                i += 1
            break
        i += 1

    return pathNodes, obstacles, start, goal, MAX_X, MAX_Y

if __name__ == "__main__":
    filename = sys.argv[1] if len(sys.argv) > 1 else "dwa.txt"
    pathNodes, obstacles, start, goal, MAX_X, MAX_Y = read_path_file(filename)
    plot_smoothed_path(pathNodes, obstacles, start, goal, MAX_X, MAX_Y)
