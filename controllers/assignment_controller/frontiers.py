from SLAM.mapping import in_bounds

from config import MAP_SIZE_X, MAP_SIZE_Y

# Function to find the frontier points in a grid map
def find_frontier(grid_map):
    f = []
    for x in range(1, MAP_SIZE_X - 1):
        for y in range(1, MAP_SIZE_Y - 1):
            if grid_map[x][y] != 1:
                continue

            has_unknown_neighbor = any(
                in_bounds(x + dx, y + dy) and grid_map[x+dx][y+dy] == 0
                for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]
            )
            if not has_unknown_neighbor:
                continue

            f.append((x, y))
    return f
