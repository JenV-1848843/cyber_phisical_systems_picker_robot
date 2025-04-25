import numpy as np
import matplotlib.pyplot as plt
import math

from SLAM.mapping import world_to_map, map_to_world

# Function to plot the grid map
def plot_map(path, frontiers, pose, grid_map, max_size_X, map_size_y, map_width, map_height, cell_size):
    img = np.zeros((max_size_X, map_size_y, 3), dtype=np.uint8)

    img[grid_map == -2] = [255, 165, 0]     # Inflated = orange
    img[grid_map == -1] = [0, 0, 0]         # Obstacles = black
    img[grid_map == 0] = [100, 100, 100]    # Unknown = gray
    img[grid_map == 1] = [255, 255, 255]    # Free = white

    # Frontiers = red
    if frontiers:
        for fx, fy in frontiers:
            if 0 <= fx < max_size_X and 0 <= fy < map_size_y:
                img[fx, fy] = [255, 0, 0]

    # Robotposition = blue
    rx, ry = world_to_map(pose[0], pose[1], map_width, map_height, cell_size)
    if 0 <= rx < max_size_X and 0 <= ry < map_size_y:
        img[rx, ry] = [0, 255, 255]

    # Path = purple | target = green
    if path:
        for (px, py) in path:
            if 0 <= px < max_size_X and 0 <= py < map_size_y:
                img[px, py] = [128, 0, 128]
        if px == path[-1][0] and py == path[-1][1]:
            img[px, py] = [0, 255, 0]

    plt.clf()
    plt.imshow(img.transpose((1, 0, 2)), origin='lower',
               extent=[-map_width/2, map_width/2, -map_height/2, map_height/2])

    # Show grid lines without labels
    major_ticks_x = np.arange(-map_width/2, map_width/2 + cell_size, cell_size)
    major_ticks_y = np.arange(-map_height/2, map_height/2 + cell_size, cell_size)
    plt.xticks(major_ticks_x, [''] * len(major_ticks_x))  # lege labels, grid blijft
    plt.yticks(major_ticks_y, [''] * len(major_ticks_y))
    plt.grid(which='major', color='gray', linestyle='--', linewidth=0.3)

    plt.title("SLAM Map")

    # Orientation arrow of the robot
    arrow_length = 0.5
    x_dir = pose[0] + arrow_length * math.cos(pose[2])
    y_dir = pose[1] + arrow_length * math.sin(pose[2])
    plt.arrow(pose[0], pose[1], x_dir - pose[0], y_dir - pose[1],
              head_width=0.15, head_length=0.15, fc='cyan', ec='cyan')

    plt.pause(0.1)  # Comment/Uncomment to see the plot in real-time in webots
    plt.savefig("../../web/static/map.png", bbox_inches='tight') # Save the figure to a file 

# Function to log status of the robot
def log_status(pose, path, frontiers, current_target, end_target, map_width, map_height, cell_size):
    mx, my = world_to_map(pose[0], pose[1], map_width, map_height, cell_size)
    print(f'\n--- Iterationstatus ---')
    
    # Robot position
    print(f'Robot position: World = ({pose[0]:.2f}, {pose[1]:.2f}) | Map = ({mx}, {my})')
    
    # Frontiers
    print(f'Amount of frontiers: {len(frontiers)}')

    # End target
    if end_target:
        end_world = map_to_world(end_target[0], end_target[1], map_width, map_height, cell_size)
        print(f'End target: World = ({end_world[0]:.2f}, {end_world[1]:.2f}) | Map = {end_target}')
    else:
        print('End target: None')

    # Current target
    if current_target:
        curr_map = world_to_map(current_target[0], current_target[1], map_width, map_height, cell_size)
        print(f'Current target: World = ({current_target[0]:.2f}, {current_target[1]:.2f}) | Map = {curr_map}')
    else:
        print('Current target: None')

    # Status
    if not frontiers and not path:
        print('Status: No frontiers and No path — Stopping')
    elif not path:
        print('Status: WAIT on pathplanning...')
    elif path:
        print(f'Status: Following path, ({len(path)} steps to go)')
    else:
        print('Status: UKNOWN')
    print('--- End Iterationstatus ---\n')
