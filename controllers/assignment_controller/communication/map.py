import numpy as np
import requests
from io import BytesIO

MAP_SERVER = "http://localhost:5000"  # Flask server for map data

# Download map from server
def download_map(endpoint, shape, dtype):
    try:
        r = requests.get(f"{MAP_SERVER}/{endpoint}")
        if r.status_code == 200:
            return np.load(BytesIO(r.content))
    except Exception as e:
        print(f"Error with retrieving map ({endpoint}): {e}")
    return np.zeros(shape, dtype=dtype)

# Upload maps to server
def upload_maps(grid_map, obstacle_map):
    try:
        buf1, buf2 = BytesIO(), BytesIO()
        np.save(buf1, grid_map)
        np.save(buf2, obstacle_map)
        buf1.seek(0)
        buf2.seek(0)
        files = {
            'grid_map': ('grid_map.npy', buf1, 'application/octet-stream'),
            'obstacle_map': ('obstacle_map.npy', buf2, 'application/octet-stream')
        }
        r = requests.post(f"{MAP_SERVER}/map", files=files)
    except Exception as e:
        print(f"Upload error: {e}")