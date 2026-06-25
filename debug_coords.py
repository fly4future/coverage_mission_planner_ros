import math

def gps_to_meters(lat, lon, lat_origin, lon_origin):
    METERS_IN_DEGREE = 111319.9
    # Simplified conversion matching the code
    meters_in_long_degree = math.cos(lat_origin * math.pi / 180.0) * METERS_IN_DEGREE
    x = (lon - lon_origin) * meters_in_long_degree
    y = (lat - lat_origin) * METERS_IN_DEGREE
    return x, y

origin_lat = 47.397743
origin_lon = 8.545594

points = [
    (47.398743, 8.544594),
    (47.398743, 8.546594),
    (47.396743, 8.546594),
    (47.396743, 8.544594)
]

print(f"Origin: {origin_lat}, {origin_lon}")
for p in points:
    x, y = gps_to_meters(p[0], p[1], origin_lat, origin_lon)
    print(f"Point ({p[0]}, {p[1]}) -> x: {x:.3f}, y: {y:.3f}")
