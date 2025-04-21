import numpy as np
import matplotlib.pyplot as plt
import math

from SLAM.mapping import world_to_map, map_to_world

def find_frontier(grid_map, max_size_X, map_size_y):
    f = []
    for x in range(1, max_size_X - 1):
        for y in range(1, map_size_y - 1):
            if grid_map[x][y] != 1: continue
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nx, ny = x+dx, y+dy
                if 0 <= nx < max_size_X and 0 <= ny < map_size_y and grid_map[nx][ny] == 0:
                    f.append((x, y)); break
    return f

def show_map(path, frontiers, pose, grid_map, max_size_X, map_size_y, map_width, map_height, cell_size):
    img = np.zeros((max_size_X, map_size_y, 3), dtype=np.uint8)

    # Kleuren toekennen
    img[grid_map == -2] = [255, 165, 0]     # Inflated = oranje
    img[grid_map == -1] = [0, 0, 0]         # Obstakels = zwart
    img[grid_map == 0] = [100, 100, 100]    # Onbekend = grijs
    img[grid_map == 1] = [255, 255, 255]    # Vrij = wit

    # Frontiers = rood
    if frontiers:
        for fx, fy in frontiers:
            if 0 <= fx < max_size_X and 0 <= fy < map_size_y:
                img[fx, fy] = [255, 0, 0]

    # Robotpositie = lichtblauw
    rx, ry = world_to_map(pose[0], pose[1], map_width, map_height, cell_size)
    if 0 <= rx < max_size_X and 0 <= ry < map_size_y:
        img[rx, ry] = [0, 255, 255]

    # Pad = paars | einddoel = groen
    if path:
        for (px, py) in path:
            if 0 <= px < max_size_X and 0 <= py < map_size_y:
                img[px, py] = [128, 0, 128]
        if px == path[-1][0] and py == path[-1][1]:
            img[px, py] = [0, 255, 0]

    plt.clf()
    plt.imshow(img.transpose((1, 0, 2)), origin='lower',
               extent=[-map_width/2, map_width/2, -map_height/2, map_height/2])

    # Grid tonen, maar zonder labels
    major_ticks_x = np.arange(-map_width/2, map_width/2 + cell_size, cell_size)
    major_ticks_y = np.arange(-map_height/2, map_height/2 + cell_size, cell_size)
    plt.xticks(major_ticks_x, [''] * len(major_ticks_x))  # lege labels, grid blijft
    plt.yticks(major_ticks_y, [''] * len(major_ticks_y))
    plt.grid(which='major', color='gray', linestyle='--', linewidth=0.3)

    plt.title("SLAM Map")

    # Oriëntatiepijl robot
    arrow_length = 0.5
    x_dir = pose[0] + arrow_length * math.cos(pose[2])
    y_dir = pose[1] + arrow_length * math.sin(pose[2])
    plt.arrow(pose[0], pose[1], x_dir - pose[0], y_dir - pose[1],
              head_width=0.15, head_length=0.15, fc='cyan', ec='cyan')

    plt.pause(0.01)

def log_status(pose, path, frontiers, current_target, end_target, map_width, map_height, cell_size):
    mx, my = world_to_map(pose[0], pose[1], map_width, map_height, cell_size)
    print(f'\n--- Iteratiestatus ---')
    
    # Robotpositie
    print(f'Robotpositie: Wereld = ({pose[0]:.2f}, {pose[1]:.2f}) | Map = ({mx}, {my})')
    
    # Frontiers
    print(f'Frontiers gevonden: {len(frontiers)}')

    # End target
    if end_target:
        end_world = map_to_world(end_target[0], end_target[1], map_width, map_height, cell_size)
        print(f'End target: Map = {end_target} | Wereld = ({end_world[0]:.2f}, {end_world[1]:.2f})')
    else:
        print('End target: geen')

    # Current target
    if current_target:
        curr_map = world_to_map(current_target[0], current_target[1], map_width, map_height, cell_size)
        print(f'Current target: Map = {curr_map} | Wereld = ({current_target[0]:.2f}, {current_target[1]:.2f})')
    else:
        print('Current target: geen')

    # Status
    if not frontiers and not path:
        print('Status: GEEN frontiers en GEEN pad — idle bewegen')
    elif not path:
        print('Status: WACHT op padplanning...')
    elif path:
        print(f'Status: Volg pad ({len(path)} stappen over)')
    else:
        print('Status: ONBEKEND')
    print('--- Einde iteratiestatus ---\n')
