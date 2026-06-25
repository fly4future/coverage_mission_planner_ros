#!/usr/bin/env python3
import math
import rospy
import matplotlib.pyplot as plt
from mrs_coverage_planner.srv import ComputeCoveragePath, ComputeCoveragePathRequest

# --- CONFIGURATION & ORIGIN SETTINGS ---
ORIGIN_LAT = 49.228330
ORIGIN_LON = 15.225238

def gps_to_meters(lat, lon, lat_origin=ORIGIN_LAT, lon_origin=ORIGIN_LON):
    """Converts GPS coordinates to local XY meters relative to an origin."""
    METERS_IN_DEGREE = 111319.9
    meters_in_long_degree = math.cos(lat_origin * math.pi / 180.0) * METERS_IN_DEGREE
    x = (lon - lon_origin) * meters_in_long_degree
    y = (lat - lat_origin) * METERS_IN_DEGREE
    return x, y

def call_coverage_planner():
    rospy.init_node('coverage_planner_client_multi', anonymous=True)
    service_name = '/mrs_coverage_planner_node/compute_coverage_path'

    rospy.loginfo(f"Waiting for service {service_name}...")
    try:
        rospy.wait_for_service(service_name, timeout=5.0)
    except rospy.ROSException:
        rospy.logerr("Service not available. Is the node running?")
        return None, None, None

    planner_service = rospy.ServiceProxy(service_name, ComputeCoveragePath)
    req = ComputeCoveragePathRequest()

    # --- DYNAMIC TYPE EXTRACTION ---
    import genpy
    DronePointType = req._slot_types[req.__slots__.index('initial_drone_positions')].replace('[]','')
    DronePtClass = genpy.message.get_message_class(DronePointType)

    ZoneWrapperType = req._slot_types[req.__slots__.index('fly_zones')].replace('[]','')
    ZoneClass = genpy.message.get_message_class(ZoneWrapperType)

    # Introspect field and point class inside a zone structure
    temp_zone = ZoneClass()
    pts_field_name = temp_zone.__slots__[0]
    PtTypeInZone = temp_zone._slot_types[temp_zone.__slots__.index(pts_field_name)].replace('[]','')
    PtClassInZone = genpy.message.get_message_class(PtTypeInZone)

    # --- POPULATE REQUEST DATA ---

    # 1. Multiple Drone Positions (Allocates tasks for 2 drones)
    req.initial_drone_positions = [
        DronePtClass(x=49.228330, y=15.225238, z=0.0),
        DronePtClass(x=49.228400, y=15.225300, z=0.0)
    ]

    # 2. TWO Separate Fly Zones (Slightly split down the middle longitudinal axis)
    raw_fly_zones = [
        # Fly Zone 1 (Western Half)
        [
            (49.228000, 15.224800),
            (49.228600, 15.224800),
            (49.228600, 15.225600),
            (49.228000, 15.225600)
        ],
        # Fly Zone 2 (Eastern Half)
        [
            (49.228000, 15.225700),
            (49.228600, 15.225700),
            (49.228600, 15.226500),
            (49.228000, 15.226500)
        ]
    ]

    req.fly_zones = []
    for zone_coordinates in raw_fly_zones:
        zone_instance = ZoneClass()
        gps_pts = [PtClassInZone(x=lat, y=lon, z=0.0) for lat, lon in zone_coordinates]
        setattr(zone_instance, pts_field_name, gps_pts)
        req.fly_zones.append(zone_instance)

    # 3. Two No-Fly Zones (One nested safely inside each respective Fly Zone)
    raw_no_fly_zones = [
        # Obstacle inside Fly Zone 1
        [(49.228200, 15.225000), (49.228300, 15.225000), (49.228300, 15.225200), (49.228200, 15.225200)],
        # Obstacle inside Fly Zone 2
        [(49.228400, 15.226000), (49.228500, 15.226000), (49.228500, 15.226200), (49.228400, 15.226200)]
    ]

    req.no_fly_zones = []
    for obstacle_coordinates in raw_no_fly_zones:
        obstacle_instance = ZoneClass()
        gps_pts = [PtClassInZone(x=lat, y=lon, z=0.0) for lat, lon in obstacle_coordinates]
        setattr(obstacle_instance, pts_field_name, gps_pts)
        req.no_fly_zones.append(obstacle_instance)

    # 4. Global configurations and dimensions matching your matrix inputs
    req.hr_no_fly_zones = []
    req.hr_no_fly_depths = []

    # Expanded distance padding lists to prevent C++ vector allocation mismatch errors
    req.min_horizontal_distances = [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
    req.min_vertical_distances = [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]

    req.target_sweeping_height = 5.0

    try:
        rospy.loginfo("Sending multi-drone request to coverage planner node...")
        response = planner_service(req)
        if response.success:
            rospy.loginfo(f"Paths successfully generated! Received paths for {len(response.drone_paths)} drones.")
            return raw_fly_zones, raw_no_fly_zones, response.drone_paths
        else:
            rospy.logwarn(f"Planner failed: {response.message}")
            return None, None, None
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None, None, None

def visualize(fly_zones, no_fly_zones, drone_paths):
    if not fly_zones or not drone_paths:
        print("Insufficient data to generate visualization.")
        return

    plt.figure(figsize=(12, 9))

    # 1. Plot all Fly Zones (Green outline)
    for idx, zone in enumerate(fly_zones):
        poly_x, poly_y = zip(*[gps_to_meters(lat, lon) for lat, lon in zone])
        poly_x = list(poly_x) + [poly_x[0]]
        poly_y = list(poly_y) + [poly_y[0]]
        plt.plot(poly_x, poly_y, 'g--', linewidth=1.5, label='Fly Zone' if idx == 0 else "")
        plt.fill(poly_x, poly_y, 'g', alpha=0.05)

    # 2. Plot all No-Fly Zones (Red filled blocks)
    for idx, obstacle in enumerate(no_fly_zones):
        obs_x, obs_y = zip(*[gps_to_meters(lat, lon) for lat, lon in obstacle])
        obs_x = list(obs_x) + [obs_x[0]]
        obs_y = list(obs_y) + [obs_y[0]]
        plt.plot(obs_x, obs_y, 'r-', linewidth=1.5, label='No-Fly Zone (Obstacle)' if idx == 0 else "")
        plt.fill(obs_x, obs_y, 'r', alpha=0.3)

    # 3. Plot Computed Fleet Track lines (Distinct colors per active agent)
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728'] # Color palette
    for i, path in enumerate(drone_paths):
        if not path.points:
            continue
        path_x = [pt.position.x for pt in path.points]
        path_y = [pt.position.y for pt in path.points]

        c = colors[i % len(colors)]
        plt.plot(path_x, path_y, color=c, linestyle='-', linewidth=2,
                 label=f'Drone {i+1} Track', marker='o', markersize=3, alpha=0.9)

        # Mark individual takeoff deployment starts and completions
        plt.plot(path_x[0], path_y[0], color=c, marker='^', markersize=9)
        plt.plot(path_x[-1], path_y[-1], color=c, marker='s', markersize=7)

    plt.title('Multi-Drone Fleet Allocation & Energy Aware MCPP Resolution', fontsize=13)
    plt.xlabel('Easting (Local Meters)', fontsize=11)
    plt.ylabel('Northing (Local Meters)', fontsize=11)
    plt.grid(True, linestyle=':', alpha=0.5)
    plt.legend(loc='upper right')
    plt.axis('equal')
    plt.show()

if __name__ == '__main__':
    fly_z, no_fly_z, paths = call_coverage_planner()
    visualize(fly_z, no_fly_z, paths)
