from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
import threading
import json
import os
import time
import pika
import numpy as np

# ──────────────────────────────────────────────────────────────
# CONFIG
# ──────────────────────────────────────────────────────────────

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# Map settings
MAP_DIR = "maps"
GRID_FILE = os.path.join(MAP_DIR, "grid_map.npy")
OBSTACLE_FILE = os.path.join(MAP_DIR, "obstacle_map.npy")
CELL_SIZE = 0.10
MAP_WIDTH = 5.0
MAP_HEIGHT = 4.0
MAP_SIZE_X = int(MAP_WIDTH / CELL_SIZE)
MAP_SIZE_Y = int(MAP_HEIGHT / CELL_SIZE)

# RabbitMQ settings
rabbitmq_host = 'localhost'
rabbitmq_queue = 'task_queue'

# Robot initialization settings
ROBOT_IDS = {"Bobbie": -10, "Bubbie": -11}
ROBOT_START_POSES = {"Bobbie": [-1.5, 0.0, 0.0], "Bubbie": [-1.5, 1.0, 0]}  # [x, y, theta]
robot_index = 0
lock = threading.Lock()

# ──────────────────────────────────────────────────────────────
# INITIALIZE ROBOTS
# ──────────────────────────────────────────────────────────────

@app.route('/robot/initialize/<name>', methods=['GET'])
def initialize_robot(name):
    with lock:
        if name in ROBOT_IDS and name in ROBOT_START_POSES:
            return jsonify({
                "robot_id": ROBOT_IDS[name],
                "start_pose": ROBOT_START_POSES[name]
            }), 200
        else:
            return jsonify({"error": f"Robot not found with name '{name}'"}), 404

# ──────────────────────────────────────────────────────────────
# SOCKETIO FOR ROBOT UPDATES
# ──────────────────────────────────────────────────────────────

@socketio.on('status_update')
def handle_status_update(data):
    with lock:
        robot_id = data.get('robot_id')
        print(f"Received status update from robot {robot_id}: {data}")

# ──────────────────────────────────────────────────────────────
# CONNECTION TO RABBITMQ
# ──────────────────────────────────────────────────────────────

def connect_to_rabbitmq():
    max_retries = 3
    retry_delay = 5

    for attempt in range(max_retries):
        try:
            connection = pika.BlockingConnection(pika.ConnectionParameters(host=rabbitmq_host))
            print(f"Successfully connected to RabbitMQ after {attempt + 1} attempts.")
            return connection
        except pika.exceptions.AMQPConnectionError as e:
            print(f"Attempt {attempt + 1} failed to connect to RabbitMQ: {e}")
            if attempt < max_retries - 1:
                print(f"Retrying in {retry_delay} seconds...")
                time.sleep(retry_delay)
            else:
                print("Failed to connect to RabbitMQ after multiple retries.")
                return None
        except Exception as ex:
            print(f"An unexpected error occurred: {ex}")
            return None

    return None

# ──────────────────────────────────────────────────────────────
# ROUTES - WEBAPP + RabbitMQ
# ──────────────────────────────────────────────────────────────

def publish_message(queue, message):
    connection = None
    try:
        credentials = pika.PlainCredentials('cyber', 'cyber')
        parameters = pika.ConnectionParameters(host='localhost',
                                               port=5672,
                                               credentials=credentials)
        connection = pika.BlockingConnection(parameters)
        channel = connection.channel()
        channel.queue_declare(queue=queue, durable=True)
        message_json = json.dumps(message)

        channel.basic_publish(
            exchange='',
            routing_key=queue,
            properties=pika.BasicProperties(
                delivery_mode=2, # (QoS 2)
            ),
            body=message_json.encode('utf-8')
        )

        print(f" [x] Sent {message} to {queue}")
        return True  # Indicate success

    except pika.exceptions.AMQPConnectionError as e:
        print(f"Error publishing message: {e}")
        return False
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return False
    finally:
        connection.close()

@app.route('/', methods=['GET', 'POST'])
def index():
    if request.method == 'POST':
        data = request.form
        validation_errors = validate_input(data)
        if validation_errors:
            return render_template('index.html', message="Validation Error: " + "; ".join(validation_errors))


        message_dict = {
            'x_target': int(data['x_target']),
            'y_target': int(data['y_target']),
            'description': data['description']
        }


        if message_dict:
            publish_result = publish_message(rabbitmq_queue, message_dict)
            if publish_result:
                return render_template('index.html', message="Message sent successfully!")
            else:
                return render_template('index.html', message=f"Failed to send message to task qeueu.  Check the RabbitMQ server and try again.")
        else:
            return render_template('index.html', message="Please enter a valid task.")

    return render_template('index.html', message="")



# ──────────────────────────────────────────────────────────────
# ROUTES helper functions
# ──────────────────────────────────────────────────────────────

def validate_input(data):
    errors = []
    try:
        x_target = int(data.get('x_target', ''))
        if not (-1000 <= x_target <= 1000):
            errors.append("x_target must be between -1000 and 1000.")
    except ValueError:
        errors.append("x_target must be an integer.")

    try:
        y_target = int(data.get('y_target', ''))
        if not (-1000 <= y_target <= 1000):
            errors.append("y_target must be between -1000 and 1000.")
    except ValueError:
        errors.append("y_target must be an integer.")

    description = data.get('description', '')
    if not isinstance(description, str) or len(description) > 128:
        errors.append("description must be a string up to 128 characters.")

    return errors


# ──────────────────────────────────────────────────────────────
# START
# ──────────────────────────────────────────────────────────────

if __name__ == '__main__':
    socketio.run(app, host="0.0.0.0", port=5000, debug=True)
