# MAP VARIABLES
CELL_SIZE = 0.1
MAP_WIDTH = 5.0    # meters
MAP_HEIGHT = 4.4   # meters
MAP_SIZE_X = int(MAP_WIDTH / CELL_SIZE)
MAP_SIZE_Y = int(MAP_HEIGHT / CELL_SIZE)

OBSTACLE = -1
UNKNOWN = 0
FREE = 1
INFLATED = -2


# ROBOT VARIABLES/SETTINGS
TIME_STEP = 64
WHEEL_RADIUS = 0.033
WHEEL_BASE = 0.160
MAX_SPEED = 6.28
SAFETY_RADIUS = 1  # radius of cells to inflate around obstacles
OBSTACLE_THRESHOLD = 50  # Amont of times a cell needs to be occupied to be considered an obstacle

# SERVER CONFIGURATION
SERVER_URL = "http://localhost:5000"  # Flask server