
# Function to find the frontier points in a grid map
def find_frontier(grid_map, max_size_X, map_size_y):
    f = []
    for x in range(1, max_size_X - 1):
        for y in range(1, map_size_y - 1):
            if grid_map[x][y] != 1:
                continue

            has_unknown_neighbor = any(
                0 <= x+dx < max_size_X and 0 <= y+dy < map_size_y and grid_map[x+dx][y+dy] == 0
                for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]
            )
            if not has_unknown_neighbor:
                continue

            f.append((x, y))
    return f
