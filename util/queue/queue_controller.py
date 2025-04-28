import json
import pika
import time

def on_task_received(ch, method, properties, body):


    try:
        message = json.loads(body.decode('utf-8'))
        x_target = message.get('x_target')
        y_target = message.get('y_target')
        description = message.get('description')

        print(f"Received Task:")
        print(f"  ➔ x_target: {x_target}")
        print(f"  ➔ y_target: {y_target}")
        print(f"  ➔ description: {description}")

        ch.basic_ack(delivery_tag=method.delivery_tag)
        print("Message acknowledged\n")

    except json.JSONDecodeError:
        print("Failed to decode JSON. Rejecting task.")
        ch.basic_nack(delivery_tag=method.delivery_tag, requeue=False)  # Don't requeue bad messages
        #TODO: discuss this!
    except Exception as e:
        print(f"An error occurred while processing the task: {e}")
        ch.basic_nack(delivery_tag=method.delivery_tag, requeue=False)


def connect_to_task_queue(callback_function):
    connection = None
    try:
        credentials = pika.PlainCredentials('cyber', 'cyber')
        parameters = pika.ConnectionParameters(host='localhost',
                                               port=5672,
                                               credentials=credentials)
        connection = pika.BlockingConnection(parameters)
        channel = connection.channel()

        queue_name = 'task_queue'
        channel.queue_declare(queue=queue_name, durable=True)
        channel.basic_qos(prefetch_count=1)
        channel.basic_consume(queue=queue_name, on_message_callback=callback_function, auto_ack=False)

        print(f" [*] Waiting for messages from {queue_name}. To exit press CTRL+C")
        channel.start_consuming()

    except pika.exceptions.ConnectionClosedByBroker:
        print("Connection was closed by the broker. Reconnecting...")
    except pika.exceptions.AMQPConnectionError as e:
        print(f"Error connecting to RabbitMQ: {e}")
    except KeyboardInterrupt:
        print(" [x] Consumer interrupted by user. Exiting...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        if connection and connection.is_open:
            connection.close()

if __name__ == '__main__':
    connect_to_task_queue(on_task_received)
