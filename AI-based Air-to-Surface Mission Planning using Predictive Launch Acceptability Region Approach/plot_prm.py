# plot_prm.py
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Ellipse
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def parse_data(filename):
    data = {}
    with open(filename, 'r') as f:
        lines = f.readlines()

    def read_block(start_line):
        block = []
        i = start_line
        while i < len(lines) and not lines[i].startswith("END_"):
            block.append(lines[i].strip())
            i += 1
        return block, i+1

    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if line == "THREATS:":
            block, i = read_block(i+1)
            data['THREATS'] = [list(map(float, b.split(','))) for b in block]
        elif line == "SAMPLES:":
            block, i = read_block(i+1)
            data['SAMPLES'] = [list(map(float, b.split(','))) for b in block]
        elif line == "EDGES:":
            block, i = read_block(i+1)
            data['EDGES'] = [list(map(int, b.split(','))) for b in block]
        elif line == "PATH:":
            block, i = read_block(i+1)
            data['PATH'] = [list(map(float, b.split(','))) for b in block]
        else:
            i += 1
    return data

def plot_ellipsoid(ax, x, y, z, a, b, c, color='red', alpha=0.3):
    u = np.linspace(0, 2 * np.pi, 20)
    v = np.linspace(0, np.pi, 20)
    xs = a * np.outer(np.cos(u), np.sin(v)) + x
    ys = b * np.outer(np.sin(u), np.sin(v)) + y
    zs = c * np.outer(np.ones_like(u), np.cos(v)) + z
    ax.plot_surface(xs, ys, zs, color=color, alpha=alpha)

def main(filename):
    data = parse_data(filename)
    samples = np.array(data['SAMPLES'])
    path_nodes = np.array(data['PATH']) if 'PATH' in data else np.empty((0,3))
    edges = data.get('EDGES', [])
    threats = data.get('THREATS', [])

    fig = plt.figure(figsize=(12,10))
    ax = fig.add_subplot(111, projection='3d')

    # Plot all samples
    if samples.size > 0:
        ax.scatter(samples[:,0], samples[:,1], samples[:,2] if samples.shape[1]>2 else np.zeros(samples.shape[0]),
                   color='grey', s=10, alpha=0.3, label='Samples')

    # Plot edges
    for e in edges:
        s_idx, t_idx = e
        x = [samples[s_idx,0], samples[t_idx,0]]
        y = [samples[s_idx,1], samples[t_idx,1]]
        z = [samples[s_idx,2], samples[t_idx,2]] if samples.shape[1]>2 else [0,0]
        ax.plot(x, y, z, color='lightgrey', alpha=0.5)

    # Plot path
    if path_nodes.size > 0:
        ax.plot(path_nodes[:,0], path_nodes[:,1], path_nodes[:,2] if path_nodes.shape[1]>2 else np.zeros(path_nodes.shape[0]),
                'green', linewidth=3, label='Path')
        ax.scatter(path_nodes[0,0], path_nodes[0,1], path_nodes[0,2], color='blue', s=100, label='Start')
        ax.scatter(path_nodes[-1,0], path_nodes[-1,1], path_nodes[-1,2], color='yellow', s=100, label='Goal')

    # Plot threats as ellipsoids
    for t in threats:
        x, y, z, a, b, c = t
        plot_ellipsoid(ax, x, y, z, a, b, c, color='red', alpha=0.3)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D PRM Path Planning')
    ax.legend()
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 plot_prm.py prm_data.txt")
        sys.exit(1)
    main(sys.argv[1])
