import socketio

from config import SERVER_URL

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

