import matplotlib.pyplot as plt
import sys

def read_and_plot(filename="path_data.txt"):
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

    fig, ax = plt.subplots(figsize=(8, 8))
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")
    ax.set_xlim(0, MAX_X)
    ax.set_ylim(0, MAX_Y)
    ax.set_aspect('equal')

    # Sparse grid (every 50 units)
    step = max(1, int(MAX_X / 20))
    ax.set_xticks(range(0, MAX_X + 1, step))
    ax.set_yticks(range(0, MAX_Y + 1, step))
    ax.grid(color='lightgray', linestyle='-', linewidth=0.5)

    # Fully filled black obstacles
    for cx, cy, r in obstacles:
        circle = plt.Circle((cx, cy), r, color='black', fill=True)
        ax.add_patch(circle)

    if pathNodes:
        xs, ys = zip(*pathNodes)
        ax.plot(xs, ys, color='blue', linewidth=1)
        ax.scatter(xs, ys, color='red', s=20)

    if start:
        ax.scatter(start[0], start[1], color='green', s=60, label='Start')
    if goal:
        ax.scatter(goal[0], goal[1], color='orange', s=60, label='Goal')

    ax.legend()
    plt.show(block=True)

if __name__ == "__main__":
    filename = sys.argv[1] if len(sys.argv) > 1 else "path_data.txt"
    read_and_plot(filename)
