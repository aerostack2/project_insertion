import json
import sys
from pathlib import Path

def extract_path_waypoints(mission_file_path):
    """
    Reads a mission JSON (even if saved as .txt), finds any layer named 'Path',
    and returns a list of [latitude, longitude] pairs in order.
    """
    p = Path(mission_file_path)
    with p.open("r", encoding="utf-8") as f:
        data = json.load(f)

    waypoints = []
    for layer in data.get("layers", []):
        if str(layer.get("name", "")).strip().lower() == "path":
            for item in layer.get("values", []):
                if isinstance(item, (list, tuple)) and len(item) == 2:
                    waypoints.append([float(item[0]), float(item[1])])
    return waypoints

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <mission_file.json>")
        sys.exit(1)

    mission_path = sys.argv[1]
    path_waypoints = extract_path_waypoints(mission_path)

    print("Extracted waypoints:")
    print("[")
    for idx, (lat, lon) in enumerate(path_waypoints, start=1):
        print(f"[{lat}, {lon}],")
    print("]")
