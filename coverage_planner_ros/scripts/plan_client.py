#!/usr/bin/env python3
"""Smoke-test client for ~/compute_coverage_path: plans a rectangle for N drones and plots the result.

    ros2 run coverage_planner_ros plan_client.py --drones 4 --size 40 40 --end-slots --png /tmp/plan.png
"""

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import Point

from coverage_planner_ros.msg import GeoPolygon
from coverage_planner_ros.srv import ComputeCoveragePath

METERS_IN_DEGREE = 111319.5


def to_geo(x, y, origin):
    """Local metres (x east, y north) -> (lat, lon), same flat projection as the planner."""
    lon_deg = math.cos(math.radians(origin[0])) * METERS_IN_DEGREE
    return origin[0] + y / METERS_IN_DEGREE, origin[1] + x / lon_deg


def to_local(lat, lon, origin):
    lon_deg = math.cos(math.radians(origin[0])) * METERS_IN_DEGREE
    return (lon - origin[1]) * lon_deg, (lat - origin[0]) * METERS_IN_DEGREE


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--service', default='/coverage_planner/compute_coverage_path')
    ap.add_argument('--origin', type=float, nargs=2, default=[49.228330, 15.225238], metavar=('LAT', 'LON'))
    ap.add_argument('--drones', type=int, default=4)
    ap.add_argument('--size', type=float, nargs=2, default=[40.0, 40.0], metavar=('W', 'H'))
    ap.add_argument('--height', type=float, default=6.0)
    ap.add_argument('--step', type=float, default=6.0)
    ap.add_argument('--max-altitude', type=float, default=15.0)
    ap.add_argument('--end-slots', action='store_true', help='end next to the area instead of returning to start')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--png', default='', help='write a plot here (needs matplotlib)')
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('coverage_plan_client')
    cli = node.create_client(ComputeCoveragePath, args.service)
    if not cli.wait_for_service(timeout_sec=10.0):
        node.get_logger().error(f'{args.service} not available')
        return 1

    o = args.origin
    w, h = args.size
    req = ComputeCoveragePath.Request()
    req.latitude_origin, req.longitude_origin = o
    area = GeoPolygon()
    for x, y in ((0, 0), (w, 0), (w, h), (0, h)):
        lat, lon = to_geo(x, y, o)
        area.points.append(Point(x=lat, y=lon))
    req.fly_zones = [area]
    for i in range(args.drones):
        lat, lon = to_geo(-15.0, 4.0 * i, o)
        req.drone_names.append(f'uav{i + 1}')
        req.initial_drone_positions.append(Point(x=lat, y=lon))
        if args.end_slots:
            lat, lon = to_geo(w + 15.0, 4.0 * i, o)
            req.end_slots.append(Point(x=lat, y=lon))
    req.sweeping_height = args.height
    req.sweeping_step = args.step
    req.max_altitude = args.max_altitude
    req.seed = args.seed

    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=120.0)
    res = fut.result()
    if res is None:
        node.get_logger().error('no response')
        return 1
    print(f'success={res.success} seed={res.seed} message="{res.message}"')
    for name, path, b, e, has, cost in zip(res.drone_names, res.drone_paths, res.sweep_begin_indices,
                                           res.sweep_end_indices, res.has_sweep, res.costs):
        zs = [p.position.z for p in path.points]
        print(f'  {name}: {len(path.points)} points, sweep [{b}, {e}) has_sweep={has} cost={cost:.1f} '
              f'z=[{min(zs):.1f}, {max(zs):.1f}] frame={path.header.frame_id}')

    if args.png and res.success:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        xs, ys = zip(*[(x, y) for x, y in ((0, 0), (w, 0), (w, h), (0, h), (0, 0))])
        plt.plot(xs, ys, 'k-', lw=2)
        for name, path in zip(res.drone_names, res.drone_paths):
            pts = [to_local(p.position.x, p.position.y, o) for p in path.points]
            plt.plot([p[0] for p in pts], [p[1] for p in pts], '-o', ms=2, label=name)
        plt.axis('equal')
        plt.legend()
        plt.savefig(args.png, dpi=90)
        print(f'plot written to {args.png}')

    node.destroy_node()
    rclpy.shutdown()
    return 0 if res.success else 2


if __name__ == '__main__':
    sys.exit(main())
