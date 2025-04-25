import math

# Update the robot's pose based on wheel sensor readings
def update_odometry(pose, prev_left, prev_right, left_sensor, right_sensor, gyro, timestep_ms, wheel_radius, wheel_base, alpha):
    dt = timestep_ms / 1000.0

    # Encoder-based rotatie
    dl = (left_sensor.getValue() - prev_left) * wheel_radius
    dr = (right_sensor.getValue() - prev_right) * wheel_radius
    prev_left = left_sensor.getValue()
    prev_right = right_sensor.getValue()
    ds = (dl + dr) / 2.0
    dtheta_encoder = (dr - dl) / wheel_base

    # Gyroscoop-based rotatie
    angular_velocity = gyro.getValues()[2]
    dtheta_gyro = angular_velocity * dt

    # Fuseren van rotatie
    dtheta = alpha * dtheta_encoder + (1 - alpha) * dtheta_gyro
    pose[2] += dtheta
    pose[0] += ds * math.cos(pose[2])
    pose[1] += ds * math.sin(pose[2])

    return pose, prev_left, prev_right, dtheta
