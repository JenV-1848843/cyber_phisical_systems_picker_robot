import math

# Update the robot's pose based on wheel sensor readings
def update_odometry(pose, prev_left, prev_right, left_sensor, right_sensor, wheel_radius, wheel_base):
    dl = (left_sensor.getValue() - prev_left) * wheel_radius
    dr = (right_sensor.getValue() - prev_right) * wheel_radius
    prev_left = left_sensor.getValue()
    prev_right = right_sensor.getValue()
    ds = (dl + dr) / 2.0
    dtheta = (dr - dl) / wheel_base
    pose[2] += dtheta
    pose[0] += ds * math.cos(pose[2])
    pose[1] += ds * math.sin(pose[2])
    return pose, prev_left, prev_right, dtheta