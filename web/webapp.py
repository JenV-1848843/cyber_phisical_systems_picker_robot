from flask import Flask, render_template, request, jsonify, send_file
import os
import time
import pika
import numpy as np

# ──────────────────────────────────────────────────────────────
# CONFIG
# ──────────────────────────────────────────────────────────────

app = Flask(__name__)

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
# INITIALIZE MAP
# ──────────────────────────────────────────────────────────────

def initialize_empty_maps():
    os.makedirs(MAP_DIR, exist_ok=True)
    empty_grid = np.zeros((MAP_SIZE_X, MAP_SIZE_Y), dtype=np.int8)
    np.save(GRID_FILE, empty_grid)

    empty_obstacles = np.zeros((MAP_SIZE_X, MAP_SIZE_Y), dtype=np.int16)
    np.save(OBSTACLE_FILE, empty_obstacles)

# ──────────────────────────────────────────────────────────────
# ROUTES - MAP API
# ──────────────────────────────────────────────────────────────

@app.route('/map', methods=['GET'])
def get_grid_map():
    try:
        return send_file(GRID_FILE, mimetype='application/octet-stream')
    except FileNotFoundError:
        return jsonify({"error": "Grid map not found"}), 404

@app.route('/obstacles', methods=['GET'])
def get_obstacle_map():
    try:
        return send_file(OBSTACLE_FILE, mimetype='application/octet-stream')
    except FileNotFoundError:
        return jsonify({"error": "Obstacle map not found"}), 404

@app.route('/map', methods=['POST'])
def update_maps():
    grid = request.files.get("grid_map")
    obstacles = request.files.get("obstacle_map")

    if grid:
        grid.save(GRID_FILE)
    if obstacles:
        obstacles.save(OBSTACLE_FILE)

    return jsonify({"status": "Maps updated"}), 200

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
        channel.basic_publish(
            exchange='',
            routing_key=queue,
            properties=pika.BasicProperties(
                delivery_mode=2, # (QoS 2)
            ),
            body=message.encode('utf-8')
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
    """
    Handles the main web page.
    - On GET, it displays the input form.
    - On POST, it publishes the message to RabbitMQ and shows a confirmation.
    """
    if request.method == 'POST':
        message = request.form['message']
        if message:
            publish_result = publish_message(rabbitmq_queue, message) # Capture the return value
            if publish_result:
                return render_template('index.html', message=f"Message '{message}' sent to RabbitMQ!")
            else:
                return render_template('index.html', message=f"Failed to send message '{message}' to task qeueu.  Check the RabbitMQ server and try again.")
        else:
            return render_template('index.html', message="Please enter a message.")
    return render_template('index.html', message="")

# ──────────────────────────────────────────────────────────────
# START
# ──────────────────────────────────────────────────────────────

if __name__ == '__main__':
    initialize_empty_maps()
    app.run(host="0.0.0.0", port=5000, debug=True)
