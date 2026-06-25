import math

def gps_to_meters(lat, lon, lat_origin, lon_origin):
    METERS_IN_DEGREE = 111319.9
    meters_in_long_degree = math.cos(lat_origin * math.pi / 180.0) * METERS_IN_DEGREE
    x = (lon - lon_origin) * meters_in_long_degree
    y = (lat - lat_origin) * METERS_IN_DEGREE
    return x, y

origin_lat = 47.397743
origin_lon = 8.545594

# These are the coordinates from the rosservice call
# Fly zone 1
fz1 = [
    (49.230160, 15.224457),
    (49.230194, 15.225604),
    (49.228683, 15.225578),
    (49.228634, 15.224508)
]
# Fly zone 2
fz2 = [
    (49.230175, 15.222691),
    (49.230167, 15.224090),
    (49.228640, 15.224138),
    (49.228640, 15.222638)
]

print("--- Fly Zone 1 ---")
for p in fz1:
    x, y = gps_to_meters(p[0], p[1], origin_lat, origin_lon)
    print(f"Point ({p[0]}, {p[1]}) -> x: {x:.3f}, y: {y:.3f}")

print("\n--- Fly Zone 2 ---")
for p in fz2:
    x, y = gps_to_meters(p[0], p[1], origin_lat, origin_lon)
    print(f"Point ({p[0], p[1]}) -> x: {x:.3f}, y: {y:.3f}")
