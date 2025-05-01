import requests

from config import SERVER_URL

from SLAM.mapping import world_to_map


def initiate_robot(robot_name):
    # Send a GET request to the map server to initialize the robot
    response = requests.get(f"{SERVER_URL}/robot/initialize/{robot_name}")
    if response.status_code == 200:
        initialized = True
        robot_ids = response.json()["robot_ids"] 
        pose = response.json()["start_pose"]
        home_cell = world_to_map(pose[0], pose[1])
        active = response.json()["active"]
        print(f"Robot initialized: {robot_name} with pose {pose} and is active = {active}")
        return initialized, robot_ids, pose, home_cell, active
    else:
        initialized = False
        robot_ids = pose = home_cell = active = None
        print(f"Error initializing robot: {response.json()['error']}")
        return initialized, robot_ids, pose, home_cell