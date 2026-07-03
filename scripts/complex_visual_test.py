#!/usr/bin/env python3
import sys
import math
import rospy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Standard ROS Message imports to replace genpy dynamic extraction
from geometry_msgs.msg import Point, Point32, Polygon
from mrs_coverage_planner.srv import ComputeCoveragePath, ComputeCoveragePathRequest

# --- CONFIGURATION & LOCAL ORIGIN FOR VISUALIZATION ONLY ---
ORIGIN_LAT = 49.228330
ORIGIN_LON = 15.225238

HR_NO_FLY_DEPTH = 3.5  # Height restriction for HR zones in meters


def gps_to_meters(lat, lon, lat_origin=ORIGIN_LAT, lon_origin=ORIGIN_LON):
    """Converts GPS coordinates to local XY meters relative to an origin for 3D plotting."""
    METERS_IN_DEGREE = 111319.9
    meters_in_long_degree = math.cos(lat_origin * math.pi / 180.0) * METERS_IN_DEGREE
    x = (lon - lon_origin) * meters_in_long_degree
    y = (lat - lat_origin) * METERS_IN_DEGREE
    return x, y


def build_request():
    """Constructs the request payload with explicit geometric gaps."""
    req = ComputeCoveragePathRequest()

    # 1. Populating Initial Drone Locations using explicit geometry_msgs/Point
    req.initial_drone_positions = [
        Point(x=49.227900, y=15.224600, z=0.0),
        Point(x=49.228330, y=15.225238, z=0.0),
        Point(x=49.227900, y=15.226600, z=0.0),
    ]

    # Fly Zones (Geometrically separated to prevent edge collisions)
    raw_fly_zones = [
        [
            (49.228000, 15.224800),
            (49.228600, 15.224800),
            (49.228600, 15.225250),
            (49.228000, 15.225250),
            (49.228000, 15.224800),
        ],
        [
            (49.228100, 15.225350),
            (49.228700, 15.225350),
            (49.228700, 15.225850),
            (49.228100, 15.225850),
            (49.228100, 15.225350),
        ],
        [
            (49.228000, 15.225950),
            (49.228600, 15.225950),
            (49.228600, 15.226500),
            (49.228000, 15.226500),
            (49.228000, 15.225950),
        ],
    ]

    # 2. Populate Fly Zones natively using geometry_msgs/Polygon and Point32 arrays
    req.fly_zones = []
    for zone_coordinates in raw_fly_zones:
        zone_instance = Polygon()
        zone_instance.points = [
            Point32(x=lat, y=lon, z=0.0) for lat, lon in zone_coordinates
        ]
        req.fly_zones.append(zone_instance)

    if not req.fly_zones:
        rospy.logerr("Cannot plan without fly zones")
        sys.exit(1)

    # No-Fly Zones nested cleanly within the boundaries
    raw_no_fly_zones = [
        [
            (49.228150, 15.224900),
            (49.228250, 15.224900),
            (49.228250, 15.225100),
            (49.228150, 15.225100),
            (49.228150, 15.224900),
        ],
        [
            (49.228400, 15.225050),
            (49.228500, 15.225050),
            (49.228500, 15.225180),
            (49.228400, 15.225180),
            (49.228400, 15.225050),
        ],
        [
            (49.228300, 15.225500),
            (49.228450, 15.225500),
            (49.228450, 15.225700),
            (49.228300, 15.225700),
            (49.228300, 15.225500),
        ],
        [
            (49.228200, 15.226100),
            (49.228350, 15.226100),
            (49.228350, 15.226300),
            (49.228200, 15.226300),
            (49.228200, 15.226100),
        ],
    ]

    # 3. Populate No-Fly Zones
    req.no_fly_zones = []
    for obstacle_coordinates in raw_no_fly_zones:
        obstacle_instance = Polygon()
        obstacle_instance.points = [
            Point32(x=lat, y=lon, z=0.0) for lat, lon in obstacle_coordinates
        ]
        req.no_fly_zones.append(obstacle_instance)

    # Height Restricted Zone safely inside Zone 3
    raw_hr_zones = [
        [
            (49.228420, 15.226150),
            (49.228520, 15.226150),
            (49.228520, 15.226350),
            (49.228420, 15.226350),
            (49.228420, 15.226150),
        ]
    ]

    # 4. Populate Height Restricted Zones
    req.hr_no_fly_zones = []
    for hrz_coordinates in raw_hr_zones:
        hrz_instance = Polygon()
        hrz_instance.points = [
            Point32(x=lat, y=lon, z=0.0) for lat, lon in hrz_coordinates
        ]
        req.hr_no_fly_zones.append(hrz_instance)

    req.hr_no_fly_depths = [HR_NO_FLY_DEPTH]

    req.min_horizontal_distances = [6.0, 4.5, 5.0]
    req.min_vertical_distances = [3.0, 3.0, 3.0]
    req.sweeping_height = 6.0
    req.latitude_origin = ORIGIN_LAT
    req.longitude_origin = ORIGIN_LON

    return req, raw_fly_zones, raw_no_fly_zones, raw_hr_zones


def visualize_scene(
    req, fly_zones, no_fly_zones, hr_zones, drone_paths=None, is_preview=True
):
    """Renders the environment map. Shows inputs if previewing, or includes trajectories if provided."""
    fig = plt.figure(figsize=(13, 9))
    ax = fig.add_subplot(111, projection="3d")
    all_x, all_y = [], []

    # 1. Plot Fly Zones
    for idx, zone in enumerate(fly_zones):
        poly_x, poly_y = zip(*[gps_to_meters(lat, lon) for lat, lon in zone])
        poly_x, poly_y = list(poly_x) + [poly_x[0]], list(poly_y) + [poly_y[0]]
        all_x.extend(poly_x)
        all_y.extend(poly_y)
        ax.plot(
            poly_x,
            poly_y,
            [0.0] * len(poly_x),
            "g--",
            linewidth=2.0,
            label="Fly Zones" if idx == 0 else "",
        )
        ax.add_collection3d(
            Poly3DCollection(
                [list(zip(poly_x, poly_y, [0.0] * len(poly_x)))],
                alpha=0.04,
                facecolors="g",
            )
        )

    # 2. Plot No-Fly Pillars
    PILLAR_HEIGHT = 15.0
    for idx, obstacle in enumerate(no_fly_zones):
        obs_x, obs_y = zip(*[gps_to_meters(lat, lon) for lat, lon in obstacle])
        obs_x, obs_y = list(obs_x) + [obs_x[0]], list(obs_y) + [obs_y[0]]
        ax.plot(
            obs_x,
            obs_y,
            [0.0] * len(obs_x),
            "r-",
            linewidth=1.8,
            label="No-Fly Pillars" if idx == 0 else "",
        )
        ax.plot(obs_x, obs_y, [PILLAR_HEIGHT] * len(obs_x), "r-", linewidth=1.8)
        for i in range(len(obs_x) - 1):
            wall = [
                [
                    (obs_x[i], obs_y[i], 0.0),
                    (obs_x[i + 1], obs_y[i + 1], 0.0),
                    (obs_x[i + 1], obs_y[i + 1], PILLAR_HEIGHT),
                    (obs_x[i], obs_y[i], PILLAR_HEIGHT),
                ]
            ]
            ax.add_collection3d(Poly3DCollection(wall, alpha=0.2, facecolors="r"))

    # 3. Plot Height Restricted Ceilings
    for idx, hrz in enumerate(hr_zones):
        hrz_x, hrz_y = zip(*[gps_to_meters(lat, lon) for lat, lon in hrz])
        hrz_x, hrz_y = list(hrz_x) + [hrz_x[0]], list(hrz_y) + [hrz_y[0]]
        CEILING = HR_NO_FLY_DEPTH
        ax.plot(
            hrz_x,
            hrz_y,
            [0.0] * len(hrz_x),
            "b-",
            linewidth=1.5,
            label="Height-Restricted Area" if idx == 0 else "",
        )
        ax.plot(hrz_x, hrz_y, [CEILING] * len(hrz_x), "b-", linewidth=1.5)
        for i in range(len(hrz_x) - 1):
            box_wall = [
                [
                    (hrz_x[i], hrz_y[i], 0.0),
                    (hrz_x[i + 1], hrz_y[i + 1], 0.0),
                    (hrz_x[i + 1], hrz_y[i + 1], CEILING),
                    (hrz_x[i], hrz_y[i], CEILING),
                ]
            ]
            ax.add_collection3d(
                Poly3DCollection(box_wall, alpha=0.3, facecolors="#ff7f0e")
            )

    # 4. Plot Drone Trajectories if available
    if not is_preview and drone_paths:
        colors = ["#1f77b4", "#9467bd", "#2ca02c"]
        for i, path in enumerate(drone_paths):
            if not path.points:
                continue

            path_x = []
            path_y = []
            path_z = []

            # --- STEP A: Map the Initial Drone Position ---
            init_drone = req.initial_drone_positions[i]
            init_x, init_y = gps_to_meters(init_drone.x, init_drone.y)

            path_x.append(init_x)
            path_y.append(init_y)
            path_z.append(init_drone.z)

            # --- STEP B: Map the Planned Waypoints ---
            for pt in path.points:
                if (
                    pt.position.x == 0.0
                    and pt.position.y == 0.0
                    and pt.position.z == 0.0
                ):
                    continue

                x_meters, y_meters = gps_to_meters(pt.position.x, pt.position.y)

                path_x.append(x_meters)
                path_y.append(y_meters)
                path_z.append(pt.position.z)

            if len(path_x) < 2:
                continue

            c = colors[i % len(colors)]
            ax.plot(
                path_x,
                path_y,
                path_z,
                color=c,
                linestyle="-",
                linewidth=2.5,
                label=f"UAV {i+1} Track",
                marker="o",
                markersize=3,
                zorder=10,
            )

            # Mark flight start (Triangle) and termination (Square)
            ax.scatter(
                path_x[0],
                path_y[0],
                path_z[0],
                color=c,
                marker="^",
                s=140,
                edgecolors="k",
                zorder=11,
            )
            ax.scatter(
                path_x[-1],
                path_y[-1],
                path_z[-1],
                color=c,
                marker="s",
                s=110,
                edgecolors="k",
                zorder=11,
            )
    if all_x and all_y:
        margin = 20.0
        ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
        ax.set_ylim(min(all_y) - margin, max(all_y) + margin)
        ax.set_zlim(0.0, PILLAR_HEIGHT + 2.0)

    title_str = (
        "PRE-FLIGHT GEOMETRY PREVIEW (Close window to continue)"
        if is_preview
        else "FINAL RESOLVED MULTI-UAV COVERAGE TRACKS"
    )
    ax.set_title(title_str, fontsize=13, fontweight="bold")
    ax.set_xlabel("East (Meters)")
    ax.set_ylabel("North (Meters)")
    ax.set_zlabel("Altitude (Meters)")
    ax.legend(loc="upper right")
    ax.set_box_aspect([1, 1, 0.4])
    plt.show()


if __name__ == "__main__":
    rospy.init_node("coverage_planner_preflight_verifier", anonymous=True)

    # Generate the request and coordinates locally
    req, fz, nfz, hrz = build_request()

    # Connect to Service and Execute
    service_name = "/mrs_coverage_planner_node/compute_coverage_path"
    rospy.loginfo(f"Connecting to planner service: {service_name}...")
    try:
        rospy.wait_for_service(service_name, timeout=3.0)
        planner_service = rospy.ServiceProxy(service_name, ComputeCoveragePath)
        response = planner_service(req)

        if response.success:
            rospy.loginfo(
                f"Success! Received paths for {len(response.drone_paths)} drones."
            )
            # Display the final result view with trajectories included
            visualize_scene(
                req, fz, nfz, hrz, drone_paths=response.drone_paths, is_preview=False
            )
        else:
            rospy.logwarn(f"Service calculation rejected: {response.message}")

    except rospy.ServiceException as e:
        rospy.logerr(f"ROS Service connection failed: {e}")
