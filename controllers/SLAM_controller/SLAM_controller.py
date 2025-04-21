# SLAM Controller met obstacle confirmatie, herplanning, robuuste visualisatie (rechthoekige arena)
from controller import Robot
import numpy as np
import math
import matplotlib.pyplot as plt
import random
import heapq

# === Config ===
TIME_STEP = 64
WHEEL_RADIUS = 0.033
WHEEL_BASE = 0.160
MAP_WIDTH = 7.0     # meters
MAP_HEIGHT = 7.0     # meters
CELL_SIZE = 0.25
MAP_SIZE_X = int(MAP_WIDTH / CELL_SIZE)
MAP_SIZE_Y = int(MAP_HEIGHT / CELL_SIZE)
MAX_SPEED = 6.28
SAFETY_RADIUS = 0.1

robot = Robot()
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_sensor = robot.getDevice("left wheel sensor")
right_sensor = robot.getDevice("right wheel sensor")
lidar = robot.getDevice("LDS-01")

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

left_sensor.enable(TIME_STEP)
right_sensor.enable(TIME_STEP)
lidar.enable(TIME_STEP)
lidar.enablePointCloud()

# Startpositie of the robot is in the center of the map
pose = [0.0, 0.0, 0.0] # [x, y, theta]
prev_left = 0.0
prev_right = 0.0
current_target = None
grid_map = np.zeros((MAP_SIZE_X, MAP_SIZE_Y), dtype=np.int8) # -1 = obstacle, 0 = unknown, 1 = free

# Outer walls of the map are set to -1 (obstacle)
for x in range(MAP_SIZE_X):
    grid_map[x][0] = -1
    grid_map[x][MAP_SIZE_Y - 1] = -1
for y in range(MAP_SIZE_Y):
    grid_map[0][y] = -1
    grid_map[MAP_SIZE_X - 1][y] = -1

# === Functions ===

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
def in_bouds(mx, my):
    return 0 < mx < MAP_SIZE_X - 1 and 0 < my < MAP_SIZE_Y - 1

# Update the robot's pose based on wheel sensor readings
def update_odometry():
    global prev_left, prev_right, pose
    dl = (left_sensor.getValue() - prev_left) * WHEEL_RADIUS
    dr = (right_sensor.getValue() - prev_right) * WHEEL_RADIUS
    prev_left = left_sensor.getValue()
    prev_right = right_sensor.getValue()
    ds = (dl + dr) / 2.0
    dtheta = (dr - dl) / WHEEL_BASE
    pose[2] += dtheta
    pose[0] += ds * math.cos(pose[2])
    pose[1] += ds * math.sin(pose[2])
    return dtheta

# Bresenham's line algorithm to find points between two coordinates
# x0, y0 = coordinates of the robot
# x1, y1 = end of lidar ray
# Returns a list of points (x, y) between (x0, y0) and (x1, y1)
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

# Update the map based on lidar readings
def update_map():
    ranges = lidar.getRangeImage()
    fov = lidar.getFov()
    res = lidar.getHorizontalResolution()
    angle_step = fov / res

    rx, ry, rtheta = pose
    map_x, map_y = world_to_map(rx, ry)

    for i, distance in enumerate(ranges):
        if i < 10 or i > len(ranges) - 10:
            continue  # Negeer de buitenste randen van de LIDAR (te veel ruis)

        angle = rtheta + fov / 2 - i * angle_step # correcte draairichting

        # Cap de afstand (alles boven 3 meter = geen obstakel gedetecteerd)
        max_range = 3.0
        if distance == float('inf') or distance > max_range:
            distance = max_range
            hit_obstacle = False
        else:
            hit_obstacle = True

        end_x = rx + math.cos(angle) * distance
        end_y = ry + math.sin(angle) * distance
        end_mx, end_my = world_to_map(end_x, end_y)

        # Bresenham tussen robot en lidar-eindpunt
        line = bresenham(map_x, map_y, end_mx, end_my)

        for j, (x, y) in enumerate(line):
            if not in_bouds(x, y):
                break  # Stop bij buiten de kaart

            if hit_obstacle and j == len(line) - 1:
                grid_map[x][y] = -1  # Laatste punt = obstakel
            else:
                grid_map[x][y] = 1   # Andere punten = vrij


def find_frontier():
    f = []
    for x in range(1, MAP_SIZE_X - 1):
        for y in range(1, MAP_SIZE_Y - 1):
            if grid_map[x][y] != 1: continue
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nx, ny = x+dx, y+dy
                if 0 <= nx < MAP_SIZE_X and 0 <= ny < MAP_SIZE_Y and grid_map[nx][ny] == 0:
                    f.append((x, y)); break
    print(f'Amount frontiers {len(f)}')
    return f

def heuristic(a, b): return abs(a[0]-b[0]) + abs(a[1]-b[1])

def astar(start, goal, cost_map):
    dirs = [(0,1),(1,0),(-1,0),(0,-1)]
    open_set, came_from = [], {}
    g = {start: 0}; f = {start: heuristic(start, goal)}
    heapq.heappush(open_set, (f[start], start))
    while open_set:
        current = heapq.heappop(open_set)[1]
        if current == goal:
            path = []
            while current in came_from:
                path.append(current); current = came_from[current]
            return path[::-1]
        for dx, dy in dirs:
            nx, ny = current[0]+dx, current[1]+dy
            if not (0 <= nx < MAP_SIZE_X and 0 <= ny < MAP_SIZE_Y): continue
            if cost_map[nx][ny] < 0: continue
            tentative = g[current] + 1
            if tentative < g.get((nx, ny), float('inf')):
                came_from[(nx, ny)] = current
                g[(nx, ny)] = tentative
                f[(nx, ny)] = tentative + heuristic((nx, ny), goal)
                heapq.heappush(open_set, (f[(nx, ny)], (nx, ny)))
    return []

def obstacle_in_path():
    return min(lidar.getRangeImage()) < 0.3

def drive_to_target(target):
    dx, dy = target[0] - pose[0], target[1] - pose[1]
    angle = math.atan2(dy, dx)
    diff = (angle - pose[2] + math.pi) % (2*math.pi) - math.pi
    dist = math.hypot(dx, dy)
    fwd = 0.0 if abs(diff) > 0.4 else min(MAX_SPEED, dist * 8)
    turn = diff * 3
    left_motor.setVelocity(max(min(fwd - turn, MAX_SPEED), -MAX_SPEED))
    right_motor.setVelocity(max(min(fwd + turn, MAX_SPEED), -MAX_SPEED))

def show_map(path=None, frontiers=None, show_lidar=True):
    img = np.zeros((MAP_SIZE_X, MAP_SIZE_Y, 3), dtype=np.uint8)

    # Basis kleuren
    img[grid_map == -1] = [0, 0, 0]         # obstakels = zwart
    img[grid_map == 0] = [100, 100, 100]    # onbekend = grijs
    img[grid_map == 1] = [255, 255, 255]    # vrije ruimte = wit
    img[grid_map == -2] = [80, 80, 80]      # opgeblazen = donkergrijs

    # Frontiers = rood
    if frontiers:
        for fx, fy in frontiers:
            if 0 <= fx < MAP_SIZE_X and 0 <= fy < MAP_SIZE_Y:
                img[fx, fy] = [255, 0, 0]

    # Robotpositie = lichtblauw
    rx, ry = world_to_map(pose[0], pose[1])
    if 0 <= rx < MAP_SIZE_X and 0 <= ry < MAP_SIZE_Y:
        img[rx, ry] = [0, 255, 255]

    # Pad = paars
    if path:
        for (px, py) in path:
            if 0 <= px < MAP_SIZE_X and 0 <= py < MAP_SIZE_Y:
                img[px, py] = [128, 0, 128]

    plt.clf()
    plt.imshow(img.transpose((1, 0, 2)), origin='lower',
            extent=[-MAP_WIDTH/2, MAP_WIDTH/2, -MAP_HEIGHT/2, MAP_HEIGHT/2])    
    plt.title("SLAM Map")

    # ==== LIDAR visualisatie (optioneel) ====
    if show_lidar:
        ranges = lidar.getRangeImage()
        fov = lidar.getFov()
        res = lidar.getHorizontalResolution()
        angle_step = fov / res
        rx_w, ry_w, rtheta = pose
        num_rays = 20
        indices = sorted(random.sample(range(10, len(ranges) - 10), num_rays))
        for i in indices:
            distance = min(ranges[i], 3.0)
            angle = rtheta - fov / 2 + i * angle_step  # let op oriëntatie!
            end_x = rx_w + math.cos(angle) * distance
            end_y = ry_w + math.sin(angle) * distance
            plt.plot([rx_w, end_x], [ry_w, end_y], color='blue', linewidth=0.5)
            plt.scatter(end_x, end_y, color='blue', s=5)

    # === Oriëntatiepijl robot ===
    arrow_length = 0.5
    x_dir = pose[0] + arrow_length * math.cos(pose[2])
    y_dir = pose[1] + arrow_length * math.sin(pose[2])
    plt.arrow(pose[0], pose[1], x_dir - pose[0], y_dir - pose[1],
              head_width=0.15, head_length=0.15, fc='cyan', ec='cyan')


    plt.pause(0.01)


plt.ion()
path = []
while robot.step(TIME_STEP) != -1:
    # === 1. Update map voor odometry (om ruis bij draaien te voorkomen) ===
    dtheta = update_odometry()
    
    if abs(dtheta) < 0.1:
        update_map()

    pose[2] %= 2 * math.pi  # Zorg dat oriëntatie tussen 0-2π blijft

    # === 1.1. Update robotpositie in de map ===
    mx, my = world_to_map(pose[0], pose[1])
    print('\n\n=== New iteration ===')
    print(f'Pose: {pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f}')
    print(f'Map [{pose[0]:.2f}, {pose[1]:.2f}] = ({mx}, {my})')

    # === 2. Huidige celpositie bepalen ===
    cell = world_to_map(pose[0], pose[1])

    # === 3. Obstakelinflatie en frontierdetectie ===
    frontiers = find_frontier()

    # === 4. Geen frontiers en geen pad = idle beweging ===
    if not frontiers and not path:
        left_motor.setVelocity(1.5)
        right_motor.setVelocity(1.5)
        show_map()
        continue

    # === 5. Nieuw pad plannen naar frontier ===
    if frontiers and not path:
        frontiers.sort(key=lambda f: heuristic(cell, f))
        for f in frontiers:
            trial = astar(cell, f, grid_map)
            if trial:
                path = trial
                current_target = map_to_world(*path[0])
                break

    # === 6. Vermijd plotselinge botsing ===
    if obstacle_in_path():
        path = []
        current_target = None
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)

    # === 7. Volg pad stap voor stap ===
    if path:
        # Check of je dicht genoeg bent bij huidige target
        if math.hypot(pose[0] - current_target[0], pose[1] - current_target[1]) < 0.15:
            path.pop(0)
            if path:
                current_target = map_to_world(*path[0])
            else:
                current_target = None
        if current_target:
            mx, my = world_to_map(current_target[0], current_target[1])
            print(f'Current target in map: ({mx}, {my})')
            drive_to_target(current_target)
        else:
            print('No target, stopping')
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)

    else:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)

    # === 8. Visualiseer map + paden ===
    show_map(path, frontiers)

plt.ioff()
plt.show()
