#!/usr/bin/env python3
import rospy
import math
import matplotlib.pyplot as plt
from mrs_coverage_planner.srv import ComputeCoveragePath, ComputeCoveragePathRequest

# --- CONFIGURATION & LOCAL ORIGIN FOR VISUALIZATION ONLY ---
ORIGIN_LAT = 49.228330
ORIGIN_LON = 15.225238

def gps_to_meters(lat, lon, lat_origin=ORIGIN_LAT, lon_origin=ORIGIN_LON):
    """Converts GPS coordinates to local XY meters relative to an origin for 3D plotting."""
    METERS_IN_DEGREE = 111319.9
    meters_in_long_degree = math.cos(lat_origin * math.pi / 180.0) * METERS_IN_DEGREE
    x = (lon - lon_origin) * meters_in_long_degree
    y = (lat - lat_origin) * METERS_IN_DEGREE
    return x, y

def call_coverage_planner():
    rospy.init_node('coverage_planner_client_py', anonymous=True)
    service_name = '/mrs_coverage_planner_node/compute_coverage_path'

    rospy.loginfo(f"Waiting for service {service_name}...")
    try:
        rospy.wait_for_service(service_name, timeout=5.0)
    except rospy.ROSException:
        rospy.logerr("Service not available. Is the node running?")
        return None, None

    planner_service = rospy.ServiceProxy(service_name, ComputeCoveragePath)
    req = ComputeCoveragePathRequest()

    try:
        import genpy

        # 1. Populate initial drone position (Original Correct Layout: x=Lat, y=Lon)
        DronePointType = req._slot_types[req.__slots__.index('initial_drone_positions')].replace('[]','')
        DroneClass = genpy.message.get_message_class(DronePointType)
        drone_pt = DroneClass(x=49.228330, y=15.225238, z=0.0)
        req.initial_drone_positions.append(drone_pt)

        # 2. Populate fly zones dynamically
        ZoneWrapperType = req._slot_types[req.__slots__.index('fly_zones')].replace('[]','')
        ZoneClass = genpy.message.get_message_class(ZoneWrapperType)
        zone_instance = ZoneClass()

        pts_field_name = zone_instance.__slots__[0]
        PtTypeInZone = zone_instance._slot_types[zone_instance.__slots__.index(pts_field_name)].replace('[]','')
        PtClassInZone = genpy.message.get_message_class(PtTypeInZone)

        raw_coordinates = [
            (49.228000, 15.224800),
            (49.228600, 15.224800),
            (49.228600, 15.225600),
            (49.228000, 15.225600)
        ]

        # Keep original mapping layout (x=lat, y=lon) to keep the node's engine happy
        gps_points_list = []
        for lat, lon in raw_coordinates:
            gps_points_list.append(PtClassInZone(x=lat, y=lon, z=0.0))

        setattr(zone_instance, pts_field_name, gps_points_list)
        req.fly_zones.append(zone_instance)

    except Exception as e:
        rospy.logerr(f"Failed to dynamically map service request attributes: {e}")
        return None, None

    # Static primitive fields
    req.no_fly_zones = []
    req.hr_no_fly_zones = []
    req.hr_no_fly_depths = []
    req.min_horizontal_distances = [5.0]
    req.min_vertical_distances = [5.0]
    req.target_sweeping_height = 5.0

    try:
        rospy.loginfo("Sending request to coverage planner node...")
        response = planner_service(req)

        if response.success:
            rospy.loginfo("Path successfully received!")
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

    # Unpacks [(x1, y1), (x2, y2), ...] into two separate tuples: (x1, x2, ...) and (y1, y2, ...)
    poly_x, poly_y = zip(*[gps_to_meters(lat, lon) for lat, lon in raw_coordinates])
    poly_x, poly_y = list(poly_x), list(poly_y)

    poly_x.append(poly_x[0])
    poly_y.append(poly_y[0])

    plt.plot(poly_x, poly_y, 'r--', label='Fly Zone Boundary (Input GPS)', linewidth=2)
    plt.fill(poly_x, poly_y, 'r', alpha=0.1)

    # 2. Plot Output Paths (The node sets response x=Lon, y=Lat)
    for i, path in enumerate(drone_paths):

        valid_points = [
            pt.position for pt in path.points
            if not (pt.position.x == 0.0 and pt.position.y == 0.0)
        ]

        path_x, path_y = zip(*[gps_to_meters(pos.y, pos.x) for pos in valid_points])
        path_x, path_y = list(path_x), list(path_y)

        if path_x:
            plt.plot(path_x, path_y, 'b-', label=f'Drone {i+1} Sweeping Path', alpha=0.8, marker='o', markersize=4)
            plt.plot(path_x[0], path_y[0], 'g^', markersize=10, label='Path Start' if i==0 else "")
            plt.plot(path_x[-1], path_y[-1], 'rs', markersize=8, label='Path End' if i==0 else "")

    plt.title('Energy-Aware MCPP Coverage Path Optimization', fontsize=14)
    plt.xlabel('Longitude', fontsize=12)
    plt.ylabel('Latitude', fontsize=12)
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.legend(loc='best')

    # Using auto scaling helps view the trajectory properly when bounded by degrees
    plt.axis('auto')
    plt.show()

if __name__ == '__main__':
    raw_coords, paths = call_coverage_planner()
    visualize(raw_coords, paths)
