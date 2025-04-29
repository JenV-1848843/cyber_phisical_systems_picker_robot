import heapq
import math

def heuristic(a, b): return abs(a[0]-b[0]) + abs(a[1]-b[1])

def astar(start, goal, cost_map, occupancy_map, map_size_x, map_size_y):
    dirs = [(0,1),(1,0),(-1,0),(0,-1)]
    open_set, came_from = [], {}
    g = {start: 0}; f = {start: heuristic(start, goal)}
    heapq.heappush(open_set, (f[start], start))
    while open_set:
        current = heapq.heappop(open_set)[1]
        if current == goal:
            path = []
            while current in came_from:
                path.append(current); current = came_from[current]
            path = path[::-1]

            # Check if at any point the calculated path passes through an occupied corridor
            for idx, (x, y) in enumerate(path):
                if occupancy_map[x][y] != 0:
                    # Cut off all nodes of the path after passing through an occupied corridor
                    path = path[:idx]

                    # Remove up to 3 more so the robot doesn't wait right before the entrance of a corridor
                    if len(path) > 0:
                        path = path[:-1]  
                    if len(path) > 0:
                        path = path[:-1]  
                    if len(path) > 0:
                        path = path[:-1]  

                    break

            return path
        for dx, dy in dirs:
            nx, ny = current[0]+dx, current[1]+dy
            if not (0 <= nx < map_size_x and 0 <= ny < map_size_y): continue
            if cost_map[nx][ny] < 0: continue
            tentative = g[current] + cost_map[nx][ny]
            if tentative < g.get((nx, ny), float('inf')):
                came_from[(nx, ny)] = current
                g[(nx, ny)] = tentative
                f[(nx, ny)] = tentative + heuristic((nx, ny), goal)
                heapq.heappush(open_set, (f[(nx, ny)], (nx, ny)))
    return []

def drive_to_target(target, pose, left_motor, right_motor, max_speed):
    dx, dy = target[0] - pose[0], target[1] - pose[1]
    angle = math.atan2(dy, dx)
    diff = (angle - pose[2] + math.pi) % (2*math.pi) - math.pi
    dist = math.hypot(dx, dy)
    fwd = 0.0 if abs(diff) > 0.4 else min(max_speed, dist * 8)
    turn = diff * 3
    left_motor.setVelocity(max(min(fwd - turn, max_speed), -max_speed))
    right_motor.setVelocity(max(min(fwd + turn, max_speed), -max_speed))