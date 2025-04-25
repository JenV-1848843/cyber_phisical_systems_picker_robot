# === SLAM Controller for Webots ===
#
# To use this code, make sure MAP_WIDTH, MAP_HEIGHT are the same as the (rectangular) arena dimensions
# CELL_SIZE can be adjusted to change the resolution of the map 
# If you want to run the simulation use turtlebot3_burger.wbt


from controller import Robot
import numpy as np
import math
import matplotlib.pyplot as plt

# Custom modules
from SLAM.mapping import inflate_obstacles, update_map, world_to_map, map_to_world
from SLAM.navigation import drive_to_target, heuristic, astar
from SLAM.odometry import update_odometry
from SLAM.utils import find_frontier, log_status, show_map

# === Configuration ===
TIME_STEP = 64
WHEEL_RADIUS = 0.033
WHEEL_BASE = 0.160
CELL_SIZE = 0.10
MAP_WIDTH = 5.0    # meters
MAP_HEIGHT = 4.0   # meters
MAP_SIZE_X = int(MAP_WIDTH / CELL_SIZE)
MAP_SIZE_Y = int(MAP_HEIGHT / CELL_SIZE)
MAX_SPEED = 6.28
SAFETY_RADIUS = CELL_SIZE  # meters
OBSTACLE_THRESHOLD = 100  # meters


# === Robot initialization ===
robot = Robot()
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_sensor = robot.getDevice("left wheel sensor")
right_sensor = robot.getDevice("right wheel sensor")
lidar = robot.getDevice("LDS-01")

# Setup motors and sensors
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)
left_sensor.enable(TIME_STEP)
right_sensor.enable(TIME_STEP)
lidar.enable(TIME_STEP)
lidar.enablePointCloud()

gps = robot.getDevice("gps")
gps.enable(TIME_STEP)

gyro = robot.getDevice("gyro")
gyro.enable(TIME_STEP)


# === Initial state ===
pose = [-1.5, 0.0, 0.0]  # [x, y, theta] MUST BE THE SAME AS COORDINATES ROBOT
prev_left = 0.0
prev_right = 0.0

current_target = None    # Target to drive to in WORLD coordinates
end_target = None        # Final goal in MAP coordinates
path = []                # Planned path (list of MAP coordinates)

# === Map initialization ===
# Map encoding: -2 = inflated cell, -1 = obstacle, 0 = unknown, 1 = free
grid_map = np.zeros((MAP_SIZE_X, MAP_SIZE_Y), dtype=np.int8)
obstacle_map = np.zeros((MAP_SIZE_X, MAP_SIZE_Y), dtype=np.int16)


# === Visualization & Logging ===
plt.ion()
log_counter = 0
LOG_INTERVAL = 10

init_map = True

# === Main control loop ===
while robot.step(TIME_STEP) != -1:
    # 1. Update dtheta via odometry voor particle motion
    pose, prev_left, prev_right, dtheta = update_odometry(
        pose, prev_left, prev_right, left_sensor, right_sensor, gyro, TIME_STEP,
        WHEEL_RADIUS, WHEEL_BASE, alpha=0.0  # alpha = 0.0 for gryo, 1.0 for odometry
    )

    # Temporary (or backup plan) fix for odometry
    # alpha = 0.0 
    # position = gps.getValues()
    # pose[0] = alpha * pose[0] + (1 - alpha) * position[0]
    # pose[1] = alpha * pose[1] + (1 - alpha) * position[1]
    
    # 2. Update the map with lidar data
    # First three seconds of the simulation are used to initialize the map (360° lidar scan)
    # After that, the lidar will be reduced to 180° to avoid noise
    if robot.getTime() > 3:
        init_map = False

    grid_map, obstacle_map = update_map(pose, lidar, grid_map, obstacle_map, MAP_WIDTH, MAP_HEIGHT, CELL_SIZE, MAP_SIZE_X, MAP_SIZE_Y, OBSTACLE_THRESHOLD, init_map)

    # 3. Determine current robot position
    robot_position = world_to_map(pose[0], pose[1], MAP_WIDTH, MAP_HEIGHT, CELL_SIZE)

    # 4. Inflate obstacles and detect frontiers
    grid_map = inflate_obstacles(grid_map, MAP_SIZE_X, MAP_SIZE_Y, CELL_SIZE, SAFETY_RADIUS)
    
    # 5. Find frontiers to be explored
    frontiers = find_frontier(grid_map, MAP_SIZE_X, MAP_SIZE_Y)

    # 6. Plan new path to a frontier if none exists
    if frontiers and not path:
        frontiers.sort(key=lambda f: heuristic(robot_position, f))
        for f in frontiers:
            trial = astar(robot_position, f, grid_map, MAP_SIZE_X, MAP_SIZE_Y)
            if trial:
                path = trial
                end_target = path[-1]
                current_target = map_to_world(path[0][0], path[0][1], MAP_WIDTH, MAP_HEIGHT, CELL_SIZE)
                break

    # 7. Follow the planned path
    if path:

        # Check if the path is still valid
        rerouting = False
        for cell in path:
            if grid_map[cell[0], cell[1]] < 0:
                print("End target is an obstacle — stopping.")
                path = []
                current_target = None
                end_target = None
                rerouting = True
                break
        
        # If the path is invalid, find a new path
        if rerouting:
            continue

        # Move towards the current target
        if math.hypot(pose[0] - current_target[0], pose[1] - current_target[1]) < 0.15:
            path.pop(0)
            if path:
                current_target = map_to_world(path[0][0], path[0][1], MAP_WIDTH, MAP_HEIGHT, CELL_SIZE)
            else:
                current_target = None

        if current_target:
            drive_to_target(current_target, pose, left_motor, right_motor, MAX_SPEED)
        else:
            print("No target — stopping.")
            path = []
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)

    # 8. Logging and visualization
    log_counter += 1
    if log_counter >= LOG_INTERVAL:
        log_counter = 0
        log_status(pose, path, frontiers, current_target, end_target, MAP_WIDTH, MAP_HEIGHT, CELL_SIZE)
        show_map(path, frontiers, pose, grid_map, MAP_SIZE_X, MAP_SIZE_Y, MAP_WIDTH, MAP_HEIGHT, CELL_SIZE)

plt.ioff()
plt.show()
