# === SLAM Controller for Webots ===
#
# To use this code, make sure MAP_WIDTH, MAP_HEIGHT are the same as the (rectangular) arena dimensions
# CELL_SIZE can be adjusted to change the resolution of the map 
# If you want to run the simulation use turtlebot3_burger.wbt
#
# TODO: - Make a 'observation count" for each cell. Every time a that cell is observed as obstacle, increment the count and every time it is observed as free, decrement the count. If the count is higer than a certain threshold, mark it as obstacle. If the count is lower than a certain threshold, mark it as free (best integrate it in update_map() and limit counter to 255 (uint8) or use other uint...). => Results in a more accurate map
# TODO: - Implement different threads for the different modules (mapping, navigation, odometry) to improve performance?
# TODO: - Integrate communication with webserver
# TODO: - Test with a long simulation to check accuracy of only using odometry
# TODO: - Test in turtleBotWorld.wbt or make a new world to simulate warehouse
# TODO: - Test with multiple robots


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
CELL_SIZE = 0.25
MAP_WIDTH = 7.0    # meters
MAP_HEIGHT = 7.0   # meters
MAP_SIZE_X = int(MAP_WIDTH / CELL_SIZE)
MAP_SIZE_Y = int(MAP_HEIGHT / CELL_SIZE)
MAX_SPEED = 6.28
SAFETY_RADIUS = CELL_SIZE  # meters


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
pose = [0.0, 0.0, 0.0]  # [x, y, theta] MUST BE THE SAME AS COORDINATES ROBOT
prev_left = 0.0
prev_right = 0.0

current_target = None    # Target to drive to in WORLD coordinates
end_target = None        # Final goal in MAP coordinates
path = []                # Planned path (list of MAP coordinates)

# === Map initialization ===
# Map encoding: -2 = inflated cell, -1 = obstacle, 0 = unknown, 1 = free
grid_map = np.zeros((MAP_SIZE_X, MAP_SIZE_Y), dtype=np.int8)

# Set outer walls as obstacles
grid_map[0, :] = -1
grid_map[-1, :] = -1
grid_map[:, 0] = -1
grid_map[:, -1] = -1

# === Visualization & Logging ===
plt.ion()
log_counter = 0
LOG_INTERVAL = 10

replan_counter = 0
REPLAN_INTERVAL = 100

# === Main control loop ===
while robot.step(TIME_STEP) != -1:
    # 1. Update odometry and maps
    # pose, prev_left, prev_right, dtheta = update_odometry(
    #     pose, prev_left, prev_right, left_sensor, right_sensor,
    #     WHEEL_RADIUS, WHEEL_BASE
    # )

    alpha = 0.0  # sterk gewicht op odometrie
    position = gps.getValues()
    pose[0] = alpha * pose[0] + (1 - alpha) * position[0]
    pose[1] = alpha * pose[1] + (1 - alpha) * position[1]

    angular_velocity = gyro.getValues()


    pose[2] += angular_velocity[2] * TIME_STEP / 1000.0
    
    update_map(pose, lidar, grid_map, MAP_WIDTH, MAP_HEIGHT, CELL_SIZE, MAP_SIZE_X, MAP_SIZE_Y)

    # 2. Determine current cell position
    cell = world_to_map(pose[0], pose[1], MAP_WIDTH, MAP_HEIGHT, CELL_SIZE)

    # 3. Inflate obstacles and detect frontiers
    inflate_obstacles(grid_map, MAP_SIZE_X, MAP_SIZE_Y, CELL_SIZE, SAFETY_RADIUS)
    frontiers = find_frontier(grid_map, MAP_SIZE_X, MAP_SIZE_Y)

    # 4. Idle movement when no path or frontiers
    if not frontiers and not path:
        left_motor.setVelocity(1.5)
        right_motor.setVelocity(1.5)
        show_map(path, frontiers, pose, grid_map, MAP_SIZE_X, MAP_SIZE_Y, MAP_WIDTH, MAP_HEIGHT, CELL_SIZE)
        continue

    # 5. Plan new path to a frontier if none exists
    if frontiers and not path:
        frontiers.sort(key=lambda f: heuristic(cell, f))
        for f in frontiers:
            trial = astar(cell, f, grid_map, MAP_SIZE_X, MAP_SIZE_Y)
            if trial:
                path = trial
                end_target = path[-1]
                current_target = map_to_world(path[0][0], path[0][1], MAP_WIDTH, MAP_HEIGHT, CELL_SIZE)
                break

    # 6. Follow the planned path
    if path:
        # Revalidate end target
        if grid_map[end_target[0]][end_target[1]] < 0:
            print("Target invalid (obstacle/inflated). Searching for new target...")
            path = []
            current_target = None
            end_target = None
            continue

        # Replan after certain number of steps
        replan_counter += 1
        if replan_counter >= REPLAN_INTERVAL and end_target:
            print("Replanning path...")
            cell = world_to_map(pose[0], pose[1], MAP_WIDTH, MAP_HEIGHT, CELL_SIZE)
            new_path = astar(cell, end_target, grid_map, MAP_SIZE_X, MAP_SIZE_Y)
            
            if new_path:
                path = new_path
                current_target = map_to_world(path[0][0], path[0][1], MAP_WIDTH, MAP_HEIGHT, CELL_SIZE)
            else:
                print("Replanning failed — no path found.")
                path = []
                current_target = None
                end_target = None

            replan_counter = 0

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
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)

    # 7. Logging
    log_counter += 1
    if log_counter >= LOG_INTERVAL:
        log_counter = 0
        log_status(pose, path, frontiers, current_target, end_target, MAP_WIDTH, MAP_HEIGHT, CELL_SIZE)

        # 8. Visualize map and planning
        show_map(path, frontiers, pose, grid_map, MAP_SIZE_X, MAP_SIZE_Y, MAP_WIDTH, MAP_HEIGHT, CELL_SIZE)

# End visualization
plt.ioff()
plt.show()
