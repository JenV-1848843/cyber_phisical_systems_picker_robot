import pika, time
from flask import Flask, render_template, request, jsonify

app = Flask(__name__)

# RabbitMQ connection parameters
rabbitmq_host = 'localhost'
rabbitmq_queue = 'task_queue'

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

if __name__ == '__main__':
    app.run(debug=True)
