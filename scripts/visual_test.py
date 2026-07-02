#!/usr/bin/env python3
import math
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Point, Point32, Polygon
from mrs_coverage_planner.srv import ComputeCoveragePath, ComputeCoveragePathRequest

# --- CONFIGURATION & LOCAL ORIGIN FOR VISUALIZATION ONLY ---
ORIGIN_LAT = 50.228330
ORIGIN_LON = 14.225238


def gps_to_meters(lat, lon, lat_origin=ORIGIN_LAT, lon_origin=ORIGIN_LON):
    """Converts GPS coordinates to local XY meters relative to an origin for 3D plotting."""
    METERS_IN_DEGREE = 111319.9
    meters_in_long_degree = math.cos(lat_origin * math.pi / 180.0) * METERS_IN_DEGREE
    x = (lon - lon_origin) * meters_in_long_degree
    y = (lat - lat_origin) * METERS_IN_DEGREE
    return x, y


def call_coverage_planner():
    rospy.init_node("coverage_planner_client_py", anonymous=True)
    service_name = "/mrs_coverage_planner_node/compute_coverage_path"

    rospy.loginfo(f"Waiting for service {service_name}...")
    try:
        rospy.wait_for_service(service_name, timeout=5.0)
    except rospy.ROSException:
        rospy.logerr("Service not available. Is the node running?")
        return None, None

    planner_service = rospy.ServiceProxy(service_name, ComputeCoveragePath)
    req = ComputeCoveragePathRequest()

    # 1. Populate multiple initial drone positions (geometry_msgs/Point[])
    # Format: x=Lat, y=Lon, z=0.0
    req.initial_drone_positions = [
        Point(x=50.228330, y=14.225238, z=0.0),  # Drone 1
        Point(x=50.228200, y=14.224900, z=0.0),  # Drone 2 (Added variant position)
    ]

    # 2. Populate fly zones natively (geometry_msgs/Polygon[])
    raw_coordinates = [
        (50.228000, 14.224800),
        (50.228600, 14.224800),
        (50.228600, 14.225600),
        (50.228000, 14.225600),
    ]

    zone_instance = Polygon()
    # geometry_msgs/Polygon points expect Point32 primitives (float32 precision)
    zone_instance.points = [
        Point32(x=lat, y=lon, z=0.0) for lat, lon in raw_coordinates
    ]
    req.fly_zones.append(zone_instance)

    # Static primitive fields
    req.no_fly_zones = []
    req.hr_no_fly_zones = []
    req.hr_no_fly_depths = []

    # Note: These constraint arrays must scale to match the length of initial_drone_positions
    num_drones = len(req.initial_drone_positions)
    req.min_horizontal_distances = [5.0] * num_drones
    req.min_vertical_distances = [5.0] * num_drones
    req.sweeping_height = 5.0
    req.latitude_origin = ORIGIN_LAT
    req.longitude_origin = ORIGIN_LON

    import os

    debug_file = "planner_request.txt"

    rospy.loginfo(f"Saved planner request to {debug_file}")
    with open(debug_file, "w") as f:
        for field in req.__slots__:
            f.write(f"{field}:\n")
            f.write(f"{getattr(req, field)}\n\n")

    try:
        rospy.loginfo("Sending request to coverage planner node...")
        response = planner_service(req)

        if response.success:
            rospy.loginfo("Paths successfully received!")
            return raw_coordinates, response.drone_paths
        else:
            rospy.logwarn(f"Planner failed: {response.message}")
            return None, None
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None, None


def visualize(raw_coordinates, drone_paths):
    if not raw_coordinates or not drone_paths:
        print("No valid data available to plot.")
        return

    plt.figure(figsize=(10, 8))

    # Plot Fly Zone Boundary
    poly_x, poly_y = zip(*[gps_to_meters(lat, lon) for lat, lon in raw_coordinates])
    poly_x, poly_y = list(poly_x), list(poly_y)
    poly_x.append(poly_x[0])
    poly_y.append(poly_y[0])

    plt.plot(poly_x, poly_y, "r--", label="Fly Zone Boundary (Input GPS)", linewidth=2)
    plt.fill(poly_x, poly_y, "r", alpha=0.1)

    # Define color cycles for clarity among multiple drones
    colors = ["#1f77b4", "#9467bd", "#2ca02c"]

    # 2. Plot Output Paths for both drones
    for i, path in enumerate(drone_paths):
        valid_points = [
            pt.position
            for pt in path.points
            if not (pt.position.x == 0.0 and pt.position.y == 0.0)
        ]

        if not valid_points:
            continue

        path_x, path_y = zip(*[gps_to_meters(pos.x, pos.y) for pos in valid_points])
        path_x, path_y = list(path_x), list(path_y)

        if path_x:
            c = colors[i % len(colors)]
            plt.plot(
                path_x,
                path_y,
                color=c,
                linestyle="-",
                label=f"Drone {i+1} Sweeping Path",
                alpha=0.8,
                marker="o",
                markersize=4,
            )
            plt.plot(
                path_x[0],
                path_y[0],
                "g^",
                markersize=10,
                label="Path Start" if i == 0 else "",
            )
            plt.plot(
                path_x[-1],
                path_y[-1],
                "rs",
                markersize=8,
                label="Path End" if i == 0 else "",
            )

    plt.title("Energy-Aware MCPP Coverage Path Optimization", fontsize=14)
    plt.xlabel("East (Meters from Origin)", fontsize=12)
    plt.ylabel("North (Meters from Origin)", fontsize=12)
    plt.grid(True, linestyle=":", alpha=0.6)
    plt.legend(loc="best")
    plt.axis("auto")
    plt.show()


if __name__ == "__main__":
    raw_coords, paths = call_coverage_planner()
    visualize(raw_coords, paths)
