from controller import Robot
import math

# E-puck Documentatie: https://www.cyberbotics.com/doc/guide/epuck?version=R2021a

# === CONSTANTEN ===
TIME_STEP = 64
WHEEL_RADIUS = 0.0205  # meter
AXLE_LENGTH = 0.053    # afstand tussen de wielen (meter)

# === INIT ===

robot = Robot()

# Motoren
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Positie sensoren
left_encoder = robot.getDevice("left wheel sensor")
right_encoder = robot.getDevice("right wheel sensor")
left_encoder.enable(TIME_STEP)
right_encoder.enable(TIME_STEP)

# Gyroscoop
gyro = robot.getDevice("gyro")
gyro.enable(TIME_STEP)

# Init pose
x, y, theta = 0.0, 0.0, 0.0
prev_left = 0.0
prev_right = 0.0

# === FUNCTIES ===

def update_odometry():
    global x, y, theta, prev_left, prev_right
    left = left_encoder.getValue()
    right = right_encoder.getValue()

    d_left = (left - prev_left) * WHEEL_RADIUS
    d_right = (right - prev_right) * WHEEL_RADIUS
    prev_left, prev_right = left, right

    d_center = (d_left + d_right) / 2
    delta_theta = (d_right - d_left) / AXLE_LENGTH
    
    x += d_center * math.cos(theta + delta_theta / 2)
    y += d_center * math.sin(theta + delta_theta / 2)
    theta += delta_theta

while robot.step(TIME_STEP) != -1:
    update_odometry()

    # Gyro (alleen Z-as relevant)
    # gyro_data = gyro.getValues()
    # dtheta_gyro = gyro_data[2] * (TIME_STEP / 1000.0)

    # Combineer gyro en encoder (eenvoudig gemiddelde)
    # dtheta = 0.5 * dtheta_encoder + 0.5 * dtheta_gyro

    # Pose update
    # theta += dtheta
    # x += ds * math.cos(theta)
    # y += ds * math.sin(theta)

    print(f"Pose: x={x:.2f} y={y:.2f} theta={math.degrees(theta):.2f}°")
    
    # Laat robot wat bewegen
    left_motor.setVelocity(2.0)
    right_motor.setVelocity(1.0)
