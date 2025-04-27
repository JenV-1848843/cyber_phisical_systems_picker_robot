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

# Custom modules
from SLAM.mapping import inflate_obstacles, update_map, world_to_map, map_to_world
from SLAM.navigation import drive_to_target, heuristic, astar
from SLAM.odometry import update_odometry
from frontiers import find_frontier
from utils import log_status, plot_map
from communication.map import download_map, upload_maps

# ──────────────────────────────────────────────────────────────
# CONFIG
# ──────────────────────────────────────────────────────────────

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
MAP_SERVER = "http://localhost:5000"  # Flask server 


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

# Robot position
pose = [-1.5, 0.0, 0.0]  # [x, y, theta] MUST BE THE SAME AS COORDINATES ROBOT
prev_left = 0.0
prev_right = 0.0

# Map initialization from server
grid_map = download_map("map", (MAP_SIZE_X, MAP_SIZE_Y), np.int8)
obstacle_map = download_map("obstacles", (MAP_SIZE_X, MAP_SIZE_Y), np.int16)

# ──────────────────────────────────────────────────────────────
# FUNCTIONS FOR CONCURRENCY
# ──────────────────────────────────────────────────────────────

def background_logger(interval):
    global pose, path, frontiers, current_target, end_target, grid_map, obstacle_map

    while True:
        try:
            # Sleep for the specified interval
            time.sleep(interval)
            log_status(pose, path, frontiers, current_target, end_target,
                       MAP_WIDTH, MAP_HEIGHT, CELL_SIZE)
            plot_map(path, frontiers, pose, grid_map,
                     MAP_SIZE_X, MAP_SIZE_Y, MAP_WIDTH, MAP_HEIGHT, CELL_SIZE)
            upload_maps(grid_map, obstacle_map)
        except Exception as e:
            print(f"Error in background logger: {e}")

def find_path_to_frontier(args):
    robot_pos, frontier, grid_map_local, map_size_x, map_size_y = args
    from SLAM.navigation import astar  # Import within process!
    return (astar(robot_pos, frontier, grid_map_local, map_size_x, map_size_y), frontier)

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

# Start thread for logging and visualization
logger_thread = threading.Thread(target=background_logger, daemon=True, args=(0.1,))
logger_thread.start()

# Main loop
while robot.step(TIME_STEP) != -1:
    # 1. Update the robot's pose using odometry and gyroscope
    pose, prev_left, prev_right = update_odometry(
        pose, prev_left, prev_right, left_sensor, right_sensor, gyro, TIME_STEP,
        WHEEL_RADIUS, WHEEL_BASE, alpha=0.0  # alpha = 0.0 for gryoscope, 1.0 for odometry
    )

    # 2. Update the map with lidar data
    # First three seconds of the simulation are used to initialize the map (360° lidar scan)
    # After that, the lidar will be reduced to 180° (in front of robot) to avoid noise
    if robot.getTime() > 3:
        init_map = False
    grid_map, obstacle_map = update_map(pose, lidar, grid_map, obstacle_map, MAP_WIDTH, MAP_HEIGHT, CELL_SIZE, MAP_SIZE_X, MAP_SIZE_Y, OBSTACLE_THRESHOLD, init_map)

    # 3. Determine current robot position
    robot_position = world_to_map(pose[0], pose[1], MAP_WIDTH, MAP_HEIGHT, CELL_SIZE)

    # 4. Inflate obstacles and detect frontiers
    grid_map = inflate_obstacles(grid_map, MAP_SIZE_X, MAP_SIZE_Y, CELL_SIZE, SAFETY_RADIUS)
    
    # === EXPLORATION ===
    if exploring:
        frontiers = find_frontier(grid_map, MAP_SIZE_X, MAP_SIZE_Y)

        # If no frontiers are found, stop exploring
        if not frontiers:
            print("Stopping exploration — no frontiers found.")
            exploring = False
            current_target = None
            end_target = None
            path = []
            frontiers = []
            continue  # of continue, afhankelijk waar je dit hebt staan
        
        # Find shortest path to frontier with parallel processing
        if not path:
            with concurrent.futures.ProcessPoolExecutor() as executor:
                tasks = [(robot_position, f, grid_map, MAP_SIZE_X, MAP_SIZE_Y) for f in frontiers]
                results = list(executor.map(find_path_to_frontier, tasks))

            valid_paths = [(trial, frontier) for trial, frontier in results if trial]

            # If there are valid paths, choose the shortest one
            if valid_paths:
                best_path, best_frontier = min(valid_paths, key=lambda x: len(x[0]))
                path = best_path
                end_target = path[-1]
                current_target = map_to_world(path[0][0], path[0][1], MAP_WIDTH, MAP_HEIGHT, CELL_SIZE)

            # If no valid paths are found, stop exploring
            else:
                print("Stopping exploration — no valid paths found.")
                exploring = False
                current_target = None
                end_target = None
                path = []
                frontiers = []

    # 5. Follow the planned path
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

    # If there is no path, drive to start location
    else:
        end_target = world_to_map(-1.5, 0.0, MAP_WIDTH, MAP_HEIGHT, CELL_SIZE)
        path = astar(robot_position, end_target, grid_map, MAP_SIZE_X, MAP_SIZE_Y)
        current_target = map_to_world(path[0][0], path[0][1], MAP_WIDTH, MAP_HEIGHT, CELL_SIZE)

