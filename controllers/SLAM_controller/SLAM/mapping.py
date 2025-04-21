import math

# Convert world coordinates to map coordinates
# x = x-coordinate in world space
# y = y-coordinate in world space
# Returns (mx, my) = map coordinates
def world_to_map(x, y, map_width, map_height, cell_size):
    mx = int((x + map_width / 2) / cell_size)
    my = int((y + map_height / 2) / cell_size)
    return mx, my

# Convert map coordinates to world coordinates
# mx = x-coordinate in map space
# my = y-coordinate in map space
# Returns (x, y) = world coordinates
def map_to_world(mx, my, map_width, map_height, cell_size):
    x = mx * cell_size - map_width / 2 + cell_size / 2
    y = my * cell_size - map_height / 2 + cell_size / 2
    return x, y

# Check if the coordinates are within the bounds of the map
def in_bounds(mx, my, map_size_x, map_size_y):
    return 0 < mx < map_size_x - 1 and 0 < my < map_size_y - 1

# Bresenham's line algorithm to find points between two coordinates
# x0, y0 = coordinates of the robot
# x1, y1 = end of lidar ray
# Returns a list of points (x, y) between (x0, y0) and (x1, y1)
def bresenham(x0, y0, x1, y1):
    points, dx, dy = [], abs(x1 - x0), abs(y1 - y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    err = dx - dy
    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1: break
        e2 = 2 * err
        if e2 > -dy: err -= dy; x0 += sx
        if e2 < dx: err += dx; y0 += sy
    return points

# Update the map based on lidar readings
def update_map(pose, lidar, grid_map, map_width, map_height, cell_size, map_size_x, map_size_y):
    ranges = lidar.getRangeImage()
    fov = lidar.getFov()
    res = lidar.getHorizontalResolution()
    angle_step = fov / res

    rx, ry, rtheta = pose
    map_x, map_y = world_to_map(rx, ry, map_width, map_height, cell_size)

    for i, distance in enumerate(ranges):
        if i < 10 or i > len(ranges) - 10:
            continue  # Negeer de buitenste randen van de LIDAR (te veel ruis)

        angle = rtheta + fov / 2 - i * angle_step # correcte draairichting

        # Cap de afstand (alles boven 3 meter = geen obstakel gedetecteerd)
        max_range = 3.0
        if distance == float('inf') or distance > max_range:
            distance = max_range
            hit_obstacle = False
        else:
            hit_obstacle = True

        end_x = rx + math.cos(angle) * distance
        end_y = ry + math.sin(angle) * distance
        end_mx, end_my = world_to_map(end_x, end_y, map_width, map_height, cell_size)

        # Bresenham tussen robot en lidar-eindpunt
        line = bresenham(map_x, map_y, end_mx, end_my)

        for j, (x, y) in enumerate(line):
            if not in_bounds(x, y, map_size_x, map_size_y):
                break  # Stop bij buiten de kaart

            if hit_obstacle and j == len(line) - 1:
                grid_map[x][y] = -1  # Laatste punt = obstakel
            else:
                grid_map[x][y] = 1   # Andere punten = vrij

    return grid_map

def inflate_obstacles(grid_map, map_size_x, map_size_y, cell_size, safety_radius):
    for x in range(map_size_x):
        for y in range(map_size_y):
            if grid_map[x][y] == -2:
                grid_map[x][y] = 1

    radius = max(1, int(safety_radius / cell_size))
    for x in range(map_size_x):
        for y in range(map_size_y):
            if grid_map[x][y] != -1:
                continue
            
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    nx, ny = x + dx, y + dy
                    if not in_bounds(nx, ny, map_size_x, map_size_y) or grid_map[nx][ny] == -1:
                        continue
                        
                    grid_map[nx][ny] = -2  # Inflated cell
    return grid_map
