"""epuck_odometry controller."""

from controller import Robot
import math

# --- Constants ---
PI = math.pi

WHEEL_RADIUS = 0.0205  # meter
AXLE_LENGTH = 0.052    # meter

# --- Odometry update function ---
def normalize_angle(angle):
    return (angle + PI) % (2 * PI) - PI

# --- Main control loop ---
def run_robot():
    timestep = 64  # ms
    dt = timestep / 1000.0  # seconden

    max_speed = 6.28  # rad/s

    # Create Robot instance
    robot = Robot()

    # Get devices
    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")

    left_position_sensor = robot.getDevice("left wheel sensor")
    right_position_sensor = robot.getDevice("right wheel sensor")

    # Initialize motors
    left_motor.setPosition(float("inf"))
    right_motor.setPosition(float("inf"))

    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    # Enable position sensors
    left_position_sensor.enable(timestep)
    right_position_sensor.enable(timestep)

    # Wait one step to get initial encoder readings
    robot.step(timestep)
    last_position = [
        left_position_sensor.getValue(),
        right_position_sensor.getValue()
    ]

    # Robot pose: [x, y, theta]
    robot_pose = [0.0, 0.0, 0.0]

    while robot.step(timestep) != -1:
        # Get current encoder values
        ps_values = [
            left_position_sensor.getValue(),
            right_position_sensor.getValue()
        ]

        # Calculate wheel displacements
        dst_values = [
            (ps_values[0] - last_position[0]) * WHEEL_RADIUS,
            (ps_values[1] - last_position[1]) * WHEEL_RADIUS
        ]

        # Update last encoder values
        last_position = ps_values.copy()

        # Linear and angular velocity
        v = (dst_values[0] + dst_values[1]) / 2.0
        w = (dst_values[1] - dst_values[0]) / AXLE_LENGTH

        # Pose update
        dx = v * math.cos(robot_pose[2]) * dt
        dy = v * math.sin(robot_pose[2]) * dt
        dtheta = w * dt

        robot_pose[0] += dx
        robot_pose[1] += dy
        robot_pose[2] = normalize_angle(robot_pose[2] + dtheta)

        # Print pose
        print("-" * 20)
        print(f"Robot pose: x={robot_pose[0]:.4f}, y={robot_pose[1]:.4f}, theta={math.degrees(robot_pose[2]):.2f}°")

        left_motor.setVelocity(max_speed)
        right_motor.setVelocity(max_speed)

# --- Entry point ---
if __name__ == "__main__":
    run_robot()
