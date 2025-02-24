"""tiago_base_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, DistanceSensor, Motor
import time

print("hello")

TIME_STEP = 64

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
# timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# initialize devices
ps = []
# psNames = [
    # 'ps0', 'ps1', 'ps2', 'ps3',
    # 'ps4', 'ps5', 'ps6', 'ps7'
# ]

# for i in range(8):
# ps.append(robot.getDevice("wheel_right_joint"))
# ps[0].enable(TIME_STEP)
    
rightMotor = robot.getDevice('wheel_right_joint')
leftMotor = robot.getDevice('wheel_left_joint')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(1.0)
rightMotor.setVelocity(1.0)
touchSensor = robot.getDevice('base_cover_link')
touchSensor.enable(100)

reverse = False

count = 0

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    if reverse == False:
        rightMotor.setVelocity(-1.0)
        leftMotor.setVelocity(-1.0)
    else:
        rightMotor.setVelocity(1.0)
        leftMotor.setVelocity(1.0)
        
    if count < 21:
        count += 1
    
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    print(touchSensor.getValue())
    if touchSensor.getValue() == 1.0:
        if count > 20:
            reverse = not reverse
            count = 0
            
    print(count)
    print(f"velocity now is: {rightMotor.getVelocity()}")
    print()
            # rightMotor.setVelocity(rightMotor.getVelocity() * -1)
            # leftMotor.setVelocity(leftMotor.getVelocity() * -1)

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
