import socketio
import numpy as np
import config

from config import SERVER_URL, ROBOT_CORRIDOR_IDS

sio = socketio.Client()

@sio.event
def connect():
    print("Connected to the server!")

@sio.event
def disconnect():
    print("Disconnect from the server!")

def connect_to_server():
    sio.connect(SERVER_URL)

# Send status updates to the server
def send_status_update(status_update):
    try:
        sio.emit("status_update", status_update)
    except Exception as e:
        print(f"Error while sending status update: {e}")

# Send map images to the server 
def send_map_update(map_img, robot_name):
    try:
        sio.emit("map_update", {
            "robot_id": robot_name,
            "map_img": f"data:image/png;base64,{map_img}"
        })
    except Exception as e:
        print(f"Error while sending map update: {e}")

# Send map data to the server -> server sends it to other robots
def send_map_data(grid_map, obstacle_map, robot_id):
    try:
        map_payload = {
            "robot_id": robot_id,
            "map_img": grid_map.tolist(),
            "obstacle_map": obstacle_map.tolist(),
        }
        sio.emit("map_data", map_payload)
    except Exception as e:
        print(f"Error while sending map data: {e}")


# Register a callback function to handle map updates from other robots
map_update_callback = None
def register_map_update_callback(callback):
    global map_update_callback
    map_update_callback = callback

# Handle incoming map data from other robots
@sio.on("map_data")
def on_map_data(data):
    try:
        robot_id = data["robot_id"]
        received_grid = np.array(data["map_img"], dtype=np.int8)
        received_obstacles = np.array(data["obstacle_map"], dtype=np.int16)

        if map_update_callback:
            map_update_callback(received_grid, received_obstacles, robot_id)
        else:
            print("No callback registered to handle map updates.")
    except Exception as e:
        print(f"Error while processing map data: {e}")

# Send corridor updates to the server
def send_corridor_update(robot_corridor_ids):
    try:
        sio.emit("corridor_update", robot_corridor_ids)
    except Exception as e:
        print(f"Error while sending corridor update: {e}")

# Register a callback function to handle corridor updates from other robots
corridor_update_callback = None
def corridor_update_callback(callback):
    global corridor_update_callback
    corridor_update_callback = callback

# Handle incoming corridor updates from other robots
@sio.on("corridor_update")
def on_corridor_status(data):
    try:
        if corridor_update_callback:
            corridor_update_callback(data)
        else:
            print("No callback registered to handle corridor updates.")
    except Exception as e:
        print(f"Error while processing corridor status: {e}")

# @sio.on("corridorstatus")
# def get_status_update(data):
#     config.ROBOT_CORRIDOR_IDS[1] = data.get('Robot 1')
#     config.ROBOT_CORRIDOR_IDS[2] = data.get('Robot 2')
#     config.ROBOT_CORRIDOR_IDS[3] = data.get('Robot 3')
#     # print(ROBOT_CORRIDOR_IDS)