import pika
import time

def on_message(ch, method, properties, body):

    print(f"Received message: {body.decode()}")
    time.sleep(1)
    ch.basic_ack(delivery_tag=method.delivery_tag)
    print("Message acknowledged")

def main():
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
        channel.basic_consume(queue=queue_name, on_message_callback=on_message, auto_ack=False)

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
    main()
