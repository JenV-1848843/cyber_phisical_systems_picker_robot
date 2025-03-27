"""E-Puck controller"""
# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, DistanceSensor, Motor
import time

########## A* implementation credit to: #################################
# https://github.com/SakshayMahna/Robotics-Playground/blob/main/turtlebot3_ws/src/global_path_planning/scripts/algorithms/astar.py

def euclidean_distance(index, goal_index, width):
  """ Heuristic Function for A Star algorithm"""
  index_x = index % width
  index_y = int(index / width)
  goal_x = goal_index % width
  goal_y = int(goal_index / width)

  distance = (index_x - goal_x) ** 2 + (index_y - goal_y) ** 2
  return sqrt(distance)

def astar(start_index, goal_index, width, height, costmap, resolution, origin, grid_viz, previous_plan_variables):
  ''' 
  Performs A Star shortest path algorithm search on a costmap with a given start and goal node
  '''

  # create an open_list
  open_list = []

  # set to hold already processed nodes
  closed_list = set()

  # dict for mapping children to parent
  parents = dict()

  # dict for mapping g costs (travel costs) to nodes
  g_costs = dict()

  # dict for mapping f costs (heuristic + travel) to nodes
  f_costs = dict()

  # set the start's node g_cost and f_cost
  g_costs[start_index] = 0
  f_costs[start_index] = 0

  # add start node to open list
  start_cost = 0 + euclidean_distance(start_index, goal_index, width)
  open_list.append([start_index, start_cost])

  shortest_path = []

  path_found = False
  rospy.loginfo('A Star: Done with initialization')

  # Main loop, executes as long as there are still nodes inside open_list
  while open_list:

    # sort open_list according to the lowest 'g_cost' value (second element of each sublist)
    open_list.sort(key = lambda x: x[1]) 
    # extract the first element (the one with the lowest 'g_cost' value)
    current_node = open_list.pop(0)[0]

    # Close current_node to prevent from visting it again
    closed_list.add(current_node)

    # Optional: visualize closed nodes
    grid_viz.set_color(current_node,"pale yellow")

    # If current_node is the goal, exit the main loop
    if current_node == goal_index:
      path_found = True
      break

    # Get neighbors of current_node
    neighbors = find_neighbors(current_node, width, height, costmap, resolution)

    # Loop neighbors
    for neighbor_index, step_cost in neighbors:

      # Check if the neighbor has already been visited
      if neighbor_index in closed_list:
        continue

      # calculate g_cost of neighbour considering it is reached through current_node
      g_cost = g_costs[current_node] + step_cost
      h_cost = euclidean_distance(neighbor_index, goal_index, width)
      f_cost = g_cost + h_cost

      # Check if the neighbor is in open_list
      in_open_list = False
      for idx, element in enumerate(open_list):
        if element[0] == neighbor_index:
          in_open_list = True
          break

      # CASE 1: neighbor already in open_list
      if in_open_list:
        if f_cost < f_costs[neighbor_index]:
          # Update the node's g_cost and f_cost
          g_costs[neighbor_index] = g_cost
          f_costs[neighbor_index] = f_cost
          parents[neighbor_index] = current_node
          # Update the node's g_cost inside open_list
          open_list[idx] = [neighbor_index, f_cost]

      # CASE 2: neighbor not in open_list
      else:
        # Set the node's g_cost and f_cost
        g_costs[neighbor_index] = g_cost
        f_costs[neighbor_index] = f_cost
        parents[neighbor_index] = current_node
        # Add neighbor to open_list
        open_list.append([neighbor_index, f_cost])

        # Optional: visualize frontier
        grid_viz.set_color(neighbor_index,'orange')

  rospy.loginfo('AStar: Done traversing nodes in open_list')

  if not path_found:
    rospy.logwarn('AStar: No path found!')
    return shortest_path

  # Reconstruct path by working backwards from target
  if path_found:
      node = goal_index
      shortest_path.append(goal_index)
      while node != start_index:
          shortest_path.append(node)
          # get next node
          node = parents[node]
  # reverse list
  shortest_path = shortest_path[::-1]
  rospy.loginfo('AStar: Done reconstructing path')

  return shortest_path, None
#############################################################################


############# function to read sensors as shown here: ################
#https://cyberbotics.com/doc/guide/controller-programming?tab-language=python#using-sensors-and-actuators-together

# def readSensors():
    # lv = lwm.getVelocity()
    # rv = rwm.getVelocity()
    # gp = gyro.getValues()
    # return lv, rv, gp

######################################################################

print('---------------------now initializing epuck controller---------------------\n\r')

TIME_STEP = 64
MAX_SPEED = 6.28

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
psNames = [
    'ps0',
    'ps1',
    'ps2',
    'ps3',
    'ps4',
    'ps5',
    'ps6',
    'ps7'
]

# initialize distance sensors
print('now starting to initialize distance sensors')
for i in range(len(psNames)):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)
print('distance sensors initialized\n\r')

# initialize step motors
print('now starting to initialize stepmotors')
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)
print('stepmotors initialized\n\r')

print('---------------------initialization complete, now starting robot main loop---------------------\n\r')

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    # read sensor outputs
    psValues = []
    for i in range(len(psNames)):
        psValues.append(ps[i].getValue())

    # detect obstacles
    right_obstacle = psValues[0] > 80.0 or psValues[1] > 80.0 or psValues[2] > 80.0
    left_obstacle = psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0

    # initialize motor speeds at 50% of MAX_SPEED.
    leftSpeed  = 0.5 * MAX_SPEED
    rightSpeed = 0.5 * MAX_SPEED
    # modify speeds according to obstacles
    if left_obstacle:
        # turn right
        leftSpeed  = 0.5 * MAX_SPEED
        rightSpeed = -0.5 * MAX_SPEED
    elif right_obstacle:
        # turn left
        leftSpeed  = -0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

# Enter here exit cleanup code.
