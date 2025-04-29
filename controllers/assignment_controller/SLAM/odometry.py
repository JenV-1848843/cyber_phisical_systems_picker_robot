import math

from config import WHEEL_RADIUS, WHEEL_BASE, TIME_STEP

# Update the robot's pose based on wheel sensor readings
def update_odometry(pose, prev_left, prev_right, left_sensor, right_sensor, gyro, alpha):
    dt = TIME_STEP / 1000.0

    # Encoder-based rotation
    dl = (left_sensor.getValue() - prev_left) * WHEEL_RADIUS
    dr = (right_sensor.getValue() - prev_right) * WHEEL_RADIUS
    prev_left = left_sensor.getValue()
    prev_right = right_sensor.getValue()
    ds = (dl + dr) / 2.0
    dtheta_encoder = (dr - dl) / WHEEL_BASE

    # Gyroscoop-based rotation
    angular_velocity = gyro.getValues()[2]
    dtheta_gyro = angular_velocity * dt

    # Fuseren van rotation -> ONLY USE GYRO! (alpha = 0.0)
    dtheta = alpha * dtheta_encoder + (1 - alpha) * dtheta_gyro
    pose[2] += dtheta
    pose[0] += ds * math.cos(pose[2])
    pose[1] += ds * math.sin(pose[2])

    return pose, prev_left, prev_right
