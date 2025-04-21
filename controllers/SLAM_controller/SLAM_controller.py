# SLAM Controller met obstacle confirmatie, herplanning, robuuste visualisatie (rechthoekige arena)
from controller import Robot
import numpy as np
import math
import matplotlib.pyplot as plt
import random
import heapq

from SLAM.mapping import inflate_obstacles, update_map, world_to_map, map_to_world
from SLAM.navigation import drive_to_target, heuristic, astar
from SLAM.odometry import update_odometry
from SLAM.utils import find_frontier, log_status, show_map

# === Config ===
TIME_STEP = 64
WHEEL_RADIUS = 0.033
WHEEL_BASE = 0.160
CELL_SIZE = 0.25
MAP_WIDTH = 7.0    # meters
MAP_HEIGHT = 7.0     # meters
MAP_SIZE_X = int(MAP_WIDTH / CELL_SIZE)
MAP_SIZE_Y = int(MAP_HEIGHT / CELL_SIZE)
MAX_SPEED = 6.28
SAFETY_RADIUS = CELL_SIZE # meters

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

# Startposition of the robot is in the center of the map
pose = [-3.0, 0.0, 0.0] # [x, y, theta]
prev_left = 0.0
prev_right = 0.0


current_target = None # next target in path in WORLD coordinates (needed to drive to)
end_target = None # end target in MAP coordinates
path = [] # list of points in the path in MAP coordinates

# Maak een lege kaart aan met onbekende cellen
# -2 = inflated cell (obstacle avoidance), -1 = obstacle, 0 = unknown, 1 = free
grid_map = np.zeros((MAP_SIZE_X, MAP_SIZE_Y), dtype=np.int8)

# Outer walls of the map are set to -1 (obstacle)
for x in range(MAP_SIZE_X):
    grid_map[x][0] = -1
    grid_map[x][MAP_SIZE_Y - 1] = -1
for y in range(MAP_SIZE_Y):
    grid_map[0][y] = -1
    grid_map[MAP_SIZE_X - 1][y] = -1

plt.ion()
log_counter = 0
LOG_INTERVAL = 10  # aantal iteraties tussen loggen
replan_counter = 0
REPLAN_INTERVAL = 100  # aantal iteraties tussen herplannen
while robot.step(TIME_STEP) != -1:
    # === 1. Update map voor odometry (om ruis bij draaien te voorkomen) ===
    pose, prev_left, prev_right, dtheta = update_odometry(pose, prev_left, prev_right, left_sensor, right_sensor, WHEEL_RADIUS, WHEEL_BASE)
    
    if abs(dtheta) < 0.1:
        update_map(pose, lidar, grid_map, MAP_WIDTH, MAP_HEIGHT, CELL_SIZE, MAP_SIZE_X, MAP_SIZE_Y)

    pose[2] %= 2 * math.pi  # Zorg dat oriëntatie tussen 0-2π blijft

    # === 2. Huidige celpositie bepalen ===
    cell = world_to_map(pose[0], pose[1], MAP_WIDTH, MAP_HEIGHT, CELL_SIZE)

    # === 3. Obstakelinflatie en frontierdetectie ===
    inflate_obstacles(grid_map, MAP_SIZE_X, MAP_SIZE_Y, CELL_SIZE, SAFETY_RADIUS)
    frontiers = find_frontier(grid_map, MAP_SIZE_X, MAP_SIZE_Y)

    # === 4. Geen frontiers en geen pad = idle beweging ===
    if not frontiers and not path:
        left_motor.setVelocity(1.5)
        right_motor.setVelocity(1.5)
        show_map(path, frontiers, pose, grid_map, MAP_SIZE_X, MAP_SIZE_Y, MAP_WIDTH, MAP_HEIGHT, CELL_SIZE)
        continue

    # === 5. Nieuw pad plannen naar frontier ===
    if frontiers and not path:
        frontiers.sort(key=lambda f: heuristic(cell, f))
        for f in frontiers:
            trial = astar(cell, f, grid_map, MAP_SIZE_X, MAP_SIZE_Y)
            if trial:
                path = trial
                end_target = path[-1]
                current_target = map_to_world(path[0][0], path[0][1], MAP_WIDTH, MAP_HEIGHT, CELL_SIZE)
                break

    # === 7. Volg pad stap voor stap ===
    if path:
        # Controleer of end_target nog geldig is (geen obstacle of inflated)
        if grid_map[end_target[0]][end_target[1]] < 0:
            print("Einddoel ligt op een obstacle of inflated cel. Nieuw doel zoeken...")
            path = []
            current_target = None
            end_target = None
            continue  # sla deze iteratie over zodat we in de volgende opnieuw plannen

        # Herbereken pad na een aantal iteraties
        replan_counter += 1
        if replan_counter >= REPLAN_INTERVAL and end_target:
            print("Herberekenen pad naar einddoel...")
            cell = world_to_map(pose[0], pose[1], MAP_WIDTH, MAP_HEIGHT, CELL_SIZE)
            new_path = astar(cell, end_target, grid_map, MAP_SIZE_X, MAP_SIZE_Y)
            
            if new_path:
                path = new_path
                current_target = map_to_world(path[0][0], path[0][1], MAP_WIDTH, MAP_HEIGHT, CELL_SIZE)

            else:
                print("Herplanning mislukt — pad niet gevonden.")
                path = []
                current_target = None
                end_target = None
            replan_counter = 0

        # Ga naar huidige target
        if math.hypot(pose[0] - current_target[0], pose[1] - current_target[1]) < 0.15:
            path.pop(0)
            if path:
                current_target = map_to_world(path[0][0], path[0][1], MAP_WIDTH, MAP_HEIGHT, CELL_SIZE)
            else:
                current_target = None

        if current_target:
            drive_to_target(current_target, pose, left_motor, right_motor, MAX_SPEED)
        else:
            print('No target, stopping')
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)

    # === 6. Log status ===
    log_counter += 1

    if log_counter >= LOG_INTERVAL:
        log_counter = 0
        log_status(pose, path, frontiers, current_target, end_target, MAP_WIDTH, MAP_HEIGHT, CELL_SIZE)

    # === 8. Visualiseer map + paden ===
    show_map(path, frontiers, pose, grid_map, MAP_SIZE_X, MAP_SIZE_Y, MAP_WIDTH, MAP_HEIGHT, CELL_SIZE)

plt.ioff()
plt.show()
