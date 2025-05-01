import socketio

from config import SERVER_URL, ROBOT_CORRIDOR_IDS

sio = socketio.Client()

@sio.event
def connect():
    print("Verbonden met de server!")

@sio.event
def disconnect():
    print("Verbinding verbroken met de server!")


def connect_to_server():
    sio.connect(SERVER_URL)

def send_status_update(status_update):
    try:
        sio.emit("status_update", status_update)
        # print(f"📤 Status verzonden: {status_update}")
    except Exception as e:
        print(f"Fout bij verzenden status update: {e}")

def send_map_update(map_img, robot_name):
    try:
        sio.emit("map_update", {
            "robot_id": robot_name,
            "map_img": f"data:image/png;base64,{map_img}"
        })
    except Exception as e:
        print(f"Fout bij verzenden map update: {e}")

@sio.on("corridorstatus")
def get_status_update(data):
    ROBOT_CORRIDOR_IDS[1] = data.get('Robot 1')
    ROBOT_CORRIDOR_IDS[2] = data.get('Robot 2')
    ROBOT_CORRIDOR_IDS[3] = data.get('Robot 3')
    # print(ROBOT_CORRIDOR_IDS)