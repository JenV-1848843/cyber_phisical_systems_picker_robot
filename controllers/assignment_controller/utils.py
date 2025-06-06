import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use a non-interactive backend
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import math
import io
import base64
import warnings

from config import MAP_WIDTH, MAP_HEIGHT, CELL_SIZE, MAP_SIZE_X, MAP_SIZE_Y, FREE, UNKNOWN, OBSTACLE, INFLATED_ZONE1, INFLATED_ZONE2

from SLAM.mapping import world_to_map, map_to_world, get_corridor_id 

# Function to plot the grid map
def plot_map(path, frontiers, pose, grid_map, occupancy_map, robot_name):
    img = np.zeros((MAP_SIZE_X, MAP_SIZE_Y, 3), dtype=np.uint8)

    img[grid_map == UNKNOWN] = [100, 100, 100]    # Unknown = gray
    img[grid_map == FREE] = [255, 255, 255]    # Free = white
    img[grid_map == OBSTACLE] = [0, 0, 0]         # Obstacles = black
    img[grid_map == INFLATED_ZONE1] = [255, 165, 0]     # Inflated zone 1 = orange
    img[grid_map == INFLATED_ZONE2] = [255, 255, 102]     # Inflated zone 2 = yellow
    img[occupancy_map == 1] = [102, 102, 255]    # Corridor occupied by robot 1 == blue
    img[occupancy_map == 2] = [255, 102, 102]    # Corridor occupied by robot 2 == red 
    img[occupancy_map == 3] = [255, 0, 255]    # Corridor occupied by robot 3 == pink 

    # Frontiers = red
    if frontiers:
        for fx, fy in frontiers:
            if 0 <= fx < MAP_SIZE_X and 0 <= fy < MAP_SIZE_Y:
                img[fx, fy] = [255, 0, 0]

    # Robotposition = blue
    rx, ry = world_to_map(pose[0], pose[1])
    if 0 <= rx < MAP_SIZE_X and 0 <= ry < MAP_SIZE_Y:
        img[rx, ry] = [0, 255, 255]

    # Path = purple | target = green
    if path:
        for (px, py) in path:
            if 0 <= px < MAP_SIZE_X and 0 <= py < MAP_SIZE_Y:
                img[px, py] = [128, 0, 128]
        if px == path[-1][0] and py == path[-1][1]:
            img[px, py] = [0, 255, 0]

    # Highlight the corridor the respective robot is in
    # for x in range(max_size_X + 1):
    #     for y in range(map_size_y + 1):
    #         if occupancy_map[x, y] == 1:
    #             print("occupied space found")
    #         img[x, y] = [255, 255, 102]

    plt.imshow(img.transpose((1, 0, 2)), origin='lower',
               extent=[-MAP_WIDTH/2, MAP_WIDTH/2, -MAP_HEIGHT/2, MAP_HEIGHT/2])

    # Show grid lines without labels
    major_ticks_x = np.arange(-MAP_WIDTH/2, MAP_WIDTH/2 + CELL_SIZE, CELL_SIZE)
    major_ticks_y = np.arange(-MAP_HEIGHT/2, MAP_HEIGHT/2 + CELL_SIZE, CELL_SIZE)
    plt.xticks(major_ticks_x, [''] * len(major_ticks_x))  # lege labels, grid blijft
    plt.yticks(major_ticks_y, [''] * len(major_ticks_y))
    plt.grid(which='major', color='gray', linestyle='--', linewidth=0.3)

    plt.title(robot_name + " - Map")

    # Orientation arrow of the robot
    arrow_length = 0.5
    x_dir = pose[0] + arrow_length * math.cos(pose[2])
    y_dir = pose[1] + arrow_length * math.sin(pose[2])
    plt.arrow(pose[0], pose[1], x_dir - pose[0], y_dir - pose[1],
              head_width=0.15, head_length=0.15, fc='cyan', ec='cyan')
    
    # Legenda naast de figuur plaatsen
    legend_elements = [
        Patch(facecolor=(100/255, 100/255, 100/255), label='Unknown'),
        Patch(facecolor='white', label='Free', edgecolor='black'),
        Patch(facecolor='black', label='Obstacle'),
        Patch(facecolor=(255/255, 165/255, 0/255), label='Inflated zone 1'),
        Patch(facecolor=(255/255, 255/255, 102/255), label='Inflated zone 2'),
        Patch(facecolor=(255/255, 0, 0), label='Frontier'),
        Patch(facecolor=(0, 255/255, 255/255), label='Robot Position'),
        Patch(facecolor=(128/255, 0, 128/255), label='Path'),
        Patch(facecolor=(0, 255/255, 0), label='Target'),
        Patch(facecolor=(102/255, 102/255, 255/255), label='Robot 1 Corridor'),
        Patch(facecolor=(255/255, 102/255, 102/255), label='Robot 2 Corridor'),
        Patch(facecolor=(255/255, 0/255, 255/255), label='Robot 3 Corridor'),
    ]

    plt.legend(handles=legend_elements,
            loc='center left',
            bbox_to_anchor=(1.0, 0.5),
            fontsize=12,
            frameon=True)

    plt.tight_layout(rect=[0, 0, 0.85, 1])  # ruimte voor de legend

    # # Interactieve plot tonen
    # plt.ion()  # interactieve modus aan
    # plt.show()  # toon venster
    # plt.pause(0.01)  # kleine pauze voor update

    # Opslaan in buffer zoals eerder
    buffer = io.BytesIO()
    plt.savefig(buffer, format='png', bbox_inches='tight')
    buffer.seek(0)

    plt.close() # NIET gebruiken als je het venster open wilt houden

    return base64.b64encode(buffer.read()).decode('utf-8')

# Function to log status of the robot
def create_status_update(robot_name, pose, end_target, exploring, task_phase):
    mx, my = world_to_map(pose[0], pose[1])
    theta_in_degrees = math.degrees(pose[2] % (2 * math.pi))
    status_msg = ""
    
    # Determine status
    if exploring:
        status_msg = "exploring"
    elif not exploring and task_phase:
        status_msg = task_phase
    elif not exploring and not task_phase:
        status_msg = "idle"
    else:
        status_msg = "unknown"

    corridorID = get_corridor_id(pose)
    # print(f"Robot {robot_id} registered itself in corridor {corridorID}")

    status_update = {
        "robot_id": robot_name,
        "position": {
            "world": {"x": round(pose[0], 2), "y": round(pose[1], 2), "theta": round(theta_in_degrees, 2)},
            "map": {"x": mx, "y": my},
            "corridor_id": corridorID
        },
        "status": status_msg,
        "end_target": {
            "world": {"x": round(map_to_world(end_target[0], end_target[1])[0], 2),
                      "y": round(map_to_world(end_target[0], end_target[1])[1], 2)},
            "map": {"x": end_target[0], "y": end_target[1]}
        } if end_target else None,
    }

    return status_update


