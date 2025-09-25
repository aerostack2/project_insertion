import json
import sys
from pathlib import Path

def extract_path_waypoints(mission_file_path, uavid):
    """
    Reads a mission JSON (even if saved as .txt), finds any layer named 'Path',
    and returns a list of [latitude, longitude] pairs in order.
    """
    p = Path(mission_file_path)
    with p.open("r", encoding="utf-8") as f:
        data = json.load(f)

    waypoints = []
    for layer in data.get("layers", []):
        if str(layer.get("name", "")).strip().lower() == "area":
            for uv in layer["uavList"]:
                print(uv)
            if uavid in layer.get("uavList", []):
                areapath = layer["uavPath"][uavid] 
                print("UAV CHECK")
                print("AREAPATH = ")
                for wp in areapath: 
                    wp = layer["values"]
                    print(wp)
                    waypoints.append([float(wp[0]), float(wp[1]), float(layer["height"])])
                #for item in layer.get("values", []):
                #    print(item)
                #    waypoints.append([float(item[0]), float(item[1])])
    return waypoints

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <mission_file.json>")
        sys.exit(1)

    mission_path = sys.argv[1]
    uav = sys.argv[2]
    path_waypoints = extract_path_waypoints(mission_path, uav)

    print("Extracted waypoints:")
    print("[")
    for idx, (lat, lon, alt) in enumerate(path_waypoints, start=1):
        print(f"[{lat}, {lon}, 20],")
    print("]")
