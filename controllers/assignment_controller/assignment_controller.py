# === SLAM Controller for Webots ===
#
# To use this code, make sure MAP_WIDTH, MAP_HEIGHT are the same as the (rectangular) arena dimensions
# CELL_SIZE can be adjusted to change the resolution of the map 
# If you want to run the simulation use turtlebot3_burger.wbt
# In utils.py comment/uncomment 'plt.pause(0.1)' (line 55) to see the plot in real-time in webots

# ──────────────────────────────────────────────────────────────
# IMPORTS
# ──────────────────────────────────────────────────────────────
from controller import Robot
import numpy as np
import math
import threading
import time
import concurrent.futures

# Config
from config import MAP_SIZE_X, MAP_SIZE_Y, TIME_STEP, OBSTACLE

# Custom modules
from SLAM.mapping import inflate_obstacles, update_map, world_to_map, map_to_world
from SLAM.navigation import drive_to_target, astar
from SLAM.odometry import update_odometry
from frontiers import find_frontier
from utils import plot_map, create_status_update
from communication.rest import initiate_robot
from communication.sockets import connect_to_server, send_status_update, send_map_update

# ──────────────────────────────────────────────────────────────
# ROBOT INITIALIZATION
# ──────────────────────────────────────────────────────────────
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
gyro = robot.getDevice("gyro")
gyro.enable(TIME_STEP)


# Initialize the robot using the REST API
ROBOT_NAME = robot.getName()
initialized, ROBOT_ID, pose, DEFAULT_POSITION = initiate_robot(ROBOT_NAME)

# Check if the robot was initialized successfully
if not initialized:
    print(f"Robot {ROBOT_NAME} not initialized. Exiting.")
    exit(1)

connect_to_server()

prev_left = 0.0
prev_right = 0.0

# Map initialization from server
grid_map = np.zeros((MAP_SIZE_X, MAP_SIZE_Y), dtype=np.int8)
obstacle_map = np.zeros((MAP_SIZE_X, MAP_SIZE_Y), dtype=np.int16)
occupancy_map = np.zeros((MAP_SIZE_X, MAP_SIZE_Y), dtype=np.int8)

# ──────────────────────────────────────────────────────────────
# FUNCTIONS FOR CONCURRENCY
# ──────────────────────────────────────────────────────────────

def background_logger(interval):
    global ROBOT_NAME, pose, path, frontiers, current_target, end_target, grid_map, obstacle_map

    while True:
        try:
            # Sleep for the specified interval
            time.sleep(interval)
            status_update = create_status_update(ROBOT_NAME, pose, path, frontiers, current_target, end_target)
            send_status_update(status_update)
            
            map_img = plot_map(path, frontiers, pose, grid_map, occupancy_map, ROBOT_NAME)
            send_map_update(map_img, ROBOT_NAME)
        except Exception as e:
            print(f"Error in background logger: {e}")

def find_path_to_frontier(args):
    robot_pos, frontier, grid_map_local = args
    from SLAM.navigation import astar  # Import within process!
    return (astar(robot_pos, frontier, grid_map_local, occupancy_map, ROBOT_ID), frontier)

# ──────────────────────────────────────────────────────────────
# MAIN LOOP
# ──────────────────────────────────────────────────────────────

# Initialize variables
init_map = True          # Flag to indicate if the map is being initialized 
current_target = None    # Target to drive to in WORLD coordinates
end_target = None        # Final goal in MAP coordinates
path = []                # Planned path (list of MAP coordinates)
frontiers = []           # List of frontiers to explore
exploring = True         # Flag to indicate if the robot is exploring  

# === Target position ===
MANUAL_POSITION = None  # Set to None for automatic exploration
# DEFAULT_POSITION = (5, 20)
PICK_INTERVAL = 300
pick_counter = 0

# Start thread for logging and visualization
logger_thread = threading.Thread(target=background_logger, daemon=True, args=(0.2,))
logger_thread.start()

# Main loop
while robot.step(TIME_STEP) != -1:
    # 1. Update the robot's pose using odometry and gyroscope
    pose, prev_left, prev_right = update_odometry(
        pose, prev_left, prev_right, left_sensor, right_sensor, gyro, alpha=0.0)

    # 2. Update the map with lidar data
    if robot.getTime() > 3:
        init_map = False

    grid_map, obstacle_map, occupancy_map = update_map(pose, lidar, grid_map, obstacle_map, occupancy_map, init_map, ROBOT_ID)

    # 3. Determine current robot position
    robot_position = world_to_map(pose[0], pose[1])

    # 4. Inflate obstacles
    grid_map = inflate_obstacles(grid_map)

    # === HANDLE MANUAL POSITION FIRST ===
    if MANUAL_POSITION is not None and not path:
        trial = astar(robot_position, MANUAL_POSITION, grid_map, occupancy_map, ROBOT_ID)
        if trial:
            path = trial
            end_target = path[-1]
            current_target = map_to_world(path[0][0], path[0][1])

    # === EXPLORATION ===
    elif exploring:
        frontiers = find_frontier(grid_map)

        if not frontiers:
            print("Stopping exploration — no frontiers found.")
            exploring = False
            current_target = None
            end_target = None
            path = []
            frontiers = []
            continue

        if not path:
            with concurrent.futures.ProcessPoolExecutor() as executor:
                tasks = [(robot_position, f, grid_map.copy()) for f in frontiers]
                results = list(executor.map(find_path_to_frontier, tasks)) # executor.map is blocking -> wait for all tasks to finish

            valid_paths = [(trial, frontier) for trial, frontier in results if trial]

            if valid_paths:
                best_path, best_frontier = min(valid_paths, key=lambda x: len(x[0]))
                path = best_path
                end_target = path[-1]
                current_target = map_to_world(path[0][0], path[0][1])
            else:
                print("Stopping exploration — no valid paths found.")
                exploring = False
                current_target = None
                end_target = None
                path = []
                frontiers = []
                MANUAL_POSITION = (40, 36)

    # === FOLLOW PATH ===
    if path:
        rerouting = False
        for cell in path:
            if grid_map[cell[0], cell[1]] < 0:
                print("End target is an obstacle — stopping.")
                path = []
                current_target = None
                end_target = None
                rerouting = True
                break

        if rerouting:
            continue

        if math.hypot(pose[0] - current_target[0], pose[1] - current_target[1]) < 0.10:
            path.pop(0)
            if path:
                current_target = map_to_world(path[0][0], path[0][1])
            else:
                current_target = None

        if current_target:
            drive_to_target(current_target, pose, left_motor, right_motor)
        elif not current_target and MANUAL_POSITION is not None:
            # After reaching MANUAL_POSITION, pause, then go to DEFAULT_POSITION
            if pick_counter <= PICK_INTERVAL:
                left_motor.setVelocity(0.0)
                right_motor.setVelocity(0.0)
                pick_counter += 1
            else:
                trial = astar(robot_position, DEFAULT_POSITION, grid_map, occupancy_map, ROBOT_ID)
                if trial:
                    path = trial
                    end_target = path[-1]
                    current_target = map_to_world(path[0][0], path[0][1])
                MANUAL_POSITION = None
                pick_counter = 0
        else:
            print("No target — stopping.")
            path = []
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)

    # === If no path (backup): return to start ===
    else:
        end_target = DEFAULT_POSITION
        path = astar(robot_position, end_target, grid_map, occupancy_map, ROBOT_ID)
        if path:
            current_target = map_to_world(path[0][0], path[0][1])
