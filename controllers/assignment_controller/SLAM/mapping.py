import math
import numpy as np
from scipy.ndimage import grey_dilation

from config import MAP_WIDTH, MAP_HEIGHT, CELL_SIZE, MAP_SIZE_X, MAP_SIZE_Y, OBSTACLE_THRESHOLD, SAFETY_RADIUS, UNKNOWN, FREE,  OBSTACLE, INFLATED_ZONE1, INFLATED_ZONE2, ROBOT_CORRIDOR_IDS

# Convert world coordinates to map coordinates
# x = x-coordinate in world space
# y = y-coordinate in world space
# Returns (mx, my) = map coordinates
def world_to_map(x, y):
    mx = int((x + MAP_WIDTH / 2) / CELL_SIZE)
    my = int((y + MAP_HEIGHT / 2) / CELL_SIZE)
    return mx, my

# Convert map coordinates to world coordinates
# mx = x-coordinate in map space
# my = y-coordinate in map space
# Returns (x, y) = world coordinates
def map_to_world(mx, my):
    x = mx * CELL_SIZE - MAP_WIDTH / 2 + CELL_SIZE / 2
    y = my * CELL_SIZE - MAP_HEIGHT / 2 + CELL_SIZE / 2
    return x, y

# Check if the coordinates are within the bounds of the map
def in_bounds(mx, my):
    return 0 <= mx <= MAP_SIZE_X - 1 and 0 <= my <= MAP_SIZE_Y - 1

# Bresenham's line algorithm to find points between two coordinates
# x0, y0 = coordinates of the robot                
# x1, y1 = end of lidar ray
# Returns a list of points(x, y) between (x0, y0) and (x1, y1)
def bresenham(x0, y0, x1, y1):
    points, dx, dy = [], abs(x1 - x0), abs(y1 - y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    err = dx - dy
    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1: break
        e2 = 2 * err
        if e2 > -dy: err -= dy; x0 += sx
        if e2 < dx: err += dx; y0 += sy
    return points

# Function to get the cells of the corridor the robot's in
def get_corridor_cells(pose, id=None):
    if id:
        corridorCells = []

        if id == 1:
            for x in range(20, 50):
                for y in range(3, 8):
                    corridorCells.append((x, y))
        elif id == 2:
            for x in range(20, 50):
                for y in range(14, 19):
                    corridorCells.append((x, y))
        elif id == 3:
            for x in range(20, 50):
                for y in range(25, 30):
                    corridorCells.append((x, y))
        elif id == 4:
            for x in range(20, 50):
                for y in range(36, 41):
                    corridorCells.append((x, y))
        else:
            if id is not None:
                print("no valid corridor ID passes to function get_corridor_cells()")

    else:
        rx, ry = world_to_map(pose[0], pose[1])

        corridorCells = []

        if 20 <= rx <= 49:
            if 3 <= ry <= 6:
                for x in range(20, 50):
                    for y in range(3, 7):
                        corridorCells.append((x, y))
            elif 13 <= ry <= 16:
                for x in range(20, 50):
                    for y in range(13, 17):
                        corridorCells.append((x, y))
            elif 23 <= ry <= 26:
                for x in range(20, 50):
                    for y in range(23, 27):
                        corridorCells.append((x, y))
            elif 33 <= ry <= 36:
                for x in range(20, 50):
                    for y in range(33, 37):
                        corridorCells.append((x, y))

    return corridorCells

def get_corridor_id(pose):
    rx, ry = world_to_map(pose[0], pose[1])

    corridorID = None

    if 20 <= rx <= 49:
        if 3 <= ry <= 6:
            corridorID = 1
        elif 13 <= ry <= 16:
            corridorID = 2
        elif 23 <= ry <= 26:
            corridorID = 3
        elif 33 <= ry <= 36:
            corridorID = 4

    return corridorID

# Update the map based on lidar readings and occupied corridors
def update_map(pose, lidar, grid_map, obstacle_map, occupancy_map, init_map, robot_id):
    '''
    # 1: define where in the map lie occupied corridors
    '''
    occupancy_map = np.zeros((MAP_SIZE_X, MAP_SIZE_Y), dtype=np.int8)

    for key, val in ROBOT_CORRIDOR_IDS.items():
        corridorCells = get_corridor_cells(pose, val)

    # FOR OCCUPANCY MAP TESTING
    # if count <= 100:
    # corridorCells = []

    # for x in range(20, 50):
    #     for y in range(3, 7):
    #         corridorCells.append((x, y))
    
    # else:
    #     corridorCells = []

    # if not corridorCells: # if list of cells is empty --> if corridor isn't occupied
    #     occupancy_map = np.zeros((MAP_SIZE_X, MAP_SIZE_Y), dtype=np.int8)
        if corridorCells:
            for (x, y) in corridorCells:
                occupancy_map[x][y] = key

    # 1: place obstacles or free space in the map based on lidar readings and further calculations
    lidar_noise = 10 if init_map else 80 #  Reduce lidar range to 180° after initialization

    ranges = lidar.getRangeImage()
    fov = lidar.getFov()
    res = lidar.getHorizontalResolution()
    angle_step = fov / res

    rx, ry, rtheta = pose
    map_x, map_y = world_to_map(rx, ry)
    max_range = 2.0  # LIDAR range cap

    for i, distance in enumerate(ranges):
        if i < lidar_noise or i > len(ranges) - lidar_noise:
            continue  # Reduce noise in the lidar data

        angle = rtheta + fov / 2 - i * angle_step
        if distance == float('inf') or distance > max_range:
            distance = max_range
            hit_obstacle = False
        else:
            hit_obstacle = True

        end_x = rx + math.cos(angle) * distance
        end_y = ry + math.sin(angle) * distance
        end_mx, end_my = world_to_map(end_x, end_y)

        line = bresenham(map_x, map_y, end_mx, end_my)

        for j, (x, y) in enumerate(line):
            if not in_bounds(x, y):
                break

            if hit_obstacle and j == len(line) - 1:
                # Last point -> increase score of being an obstacle
                val = int(obstacle_map[x][y]) + 3
                val = min(val, 255)
                obstacle_map[x][y] = val

                if obstacle_map[x][y] >= OBSTACLE_THRESHOLD:
                    grid_map[x][y] = OBSTACLE  # If the score is higher than threshold, mark as obstacle
            else:
                # Free space -> decrease score of being an obstacle
                val = int(obstacle_map[x][y]) - 1
                val = max(val, 0)
                obstacle_map[x][y] = val

                if obstacle_map[x][y] < OBSTACLE_THRESHOLD:
                    grid_map[x][y] = FREE  # If the score is lower than threshold, mark as free space

    return grid_map, obstacle_map, occupancy_map

# Function to inflate obstacles in the map
# A mask is used to increase speed in stead of for loops
def inflate_obstacles(grid_map, frontiers):
    inflated_map = np.copy(grid_map)

    # Create a mask for the obstacles
    obstacle_mask = (grid_map == OBSTACLE).astype(np.uint8)

    # Create a mask and safe zone for the frontiers -> otherwise frontiers will be overwritten
    protection_mask = np.zeros_like(grid_map, dtype=bool)
    for fx, fy in frontiers:
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                nx, ny = fx + dx, fy + dy
                if 0 <= nx < grid_map.shape[0] and 0 <= ny < grid_map.shape[1]:
                    protection_mask[nx, ny] = True

    # INFLATED ZONE 1
    structure1 = np.ones((3, 3), dtype=np.uint8)
    inflated_mask1 = grey_dilation(obstacle_mask, footprint=structure1)

    mask1 = (inflated_mask1 == 1) & (inflated_map == FREE) & (~protection_mask)
    inflated_map[mask1] = INFLATED_ZONE1

    # INFLATED ZONE 2
    structure2 = np.ones((5, 5), dtype=np.uint8)
    inflated_mask2 = grey_dilation(obstacle_mask, footprint=structure2)

    mask2 = (inflated_mask2 == 1) & (inflated_map == FREE) & (~protection_mask)
    inflated_map[mask2] = INFLATED_ZONE2

    return inflated_map
