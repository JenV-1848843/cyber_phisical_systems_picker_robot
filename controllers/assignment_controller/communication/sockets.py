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

@sio.on("corridorstatus")
def get_status_update(data):
    ROBOT_CORRIDOR_IDS[1] = data.get('Bobbie')
    ROBOT_CORRIDOR_IDS[2] = data.get('Bubbie')
    print(ROBOT_CORRIDOR_IDS)