import numpy as np
import cv2
import pyvista as pv

def plot_map(png_file, txt_file):
    with open(txt_file) as f:
        bx, by, bz = map(float, f.readline().split())
        sx_start, sy_start, sz_start = map(float, f.readline().split())
        num_goals = int(f.readline())
        goals = [list(map(float, f.readline().split())) for _ in range(num_goals)]
        rows, cols = map(int, f.readline().split())
        threats = []
        path_coords = []
        for line in f:
            vals = list(map(float, line.split()))
            if not vals:
                continue
            if len(vals) == 5:
                threats.append(vals)
            elif len(vals) == 3:
                path_coords.append(vals)

    heightmap = cv2.imread(png_file, cv2.IMREAD_GRAYSCALE)
    hx, hy = heightmap.shape
    Z = heightmap.astype(float)
    x = np.linspace(0, bx, hy)
    y = np.linspace(0, by, hx)
    X, Y = np.meshgrid(x, y)
    grid = pv.StructuredGrid(X, Y, Z)

    plotter = pv.Plotter()
    surf = plotter.add_mesh(grid, cmap="plasma", opacity=0.6, show_edges=False)

    for t in threats:
        tx, ty, tz, R, H = t
        cyl = pv.Cylinder(center=(tx, ty, tz), direction=(0,0,1), radius=R, height=H)
        plotter.add_mesh(cyl, color='b', opacity=0.7)

    plotter.add_mesh(pv.Sphere(radius=bx*0.01, center=(sx_start, sy_start, sz_start)), color='g')
    for g in goals:
        plotter.add_mesh(pv.Sphere(radius=bx*0.01, center=g), color='y')

    for coord in path_coords:
        plotter.add_mesh(pv.Sphere(radius=bx*0.01, center=coord), color='r')

    if len(path_coords) > 1:
        for i in range(len(path_coords) - 1):
            start = path_coords[i]
            end = path_coords[i + 1]
            line = pv.Line(start, end)
            plotter.add_mesh(line, color='r', line_width=3)

    plotter.add_axes()
    plotter.show_bounds(grid='back', location='outer')

    def update_opacity(value):
        surf.GetProperty().SetOpacity(value)
    plotter.add_slider_widget(update_opacity, [0.1, 1.0], value=0.6, title='Surface Opacity')

    plotter.show()

plot_map("heightmap.png", "map_data.txt")
