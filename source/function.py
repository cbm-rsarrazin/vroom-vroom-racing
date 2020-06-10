import math


def reward_function(params):
    # parameters
    score_max_direction = 5
    score_max_complete = 20
    prediction_weight = 0.7
    speed_max = 4

    x = params['x']
    y = params['y']
    progress = params['progress']
    speed = params['speed']
    steering_angle = params['steering_angle']
    steps = params['steps']
    track_width = params['track_width']
    heading = params['heading']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    is_offtrack = params['is_offtrack']

    clothest_0 = waypoints[closest_waypoints[0]]
    clothest_1 = waypoints[closest_waypoints[1]]

    source_idx = closest_waypoints[0]
    target_distance_view, dist, nearest = compute_distance_view(x, y, source_idx, waypoints, track_width)
    target_idx = source_idx + target_distance_view

    source = waypoints[source_idx % len(waypoints)]
    target = waypoints[target_idx % len(waypoints)]

    # best dir
    best_dir = nor(atan2_deg(x, y, target[0], target[1]))
    heading = nor(heading)
    steering = nor(heading + steering_angle)

    speed_ratio = speed / speed_max
    prediction_ratio = (1 - prediction_weight) + speed_ratio * prediction_weight
    predicted = nor(heading + prediction_ratio * steering_angle)

    direction_diff = math.fabs(angle_min_diff(predicted, best_dir))
    direction_diff_ratio = pow(float(1 - direction_diff / 180), 2)

    # reward
    reward = round(score_max_direction * direction_diff_ratio, 1)

    if progress == 100:
        reward += score_max_complete
    if is_offtrack:
        reward = 0.0

    log(waypoints,
        closest_waypoints,
        track_width,
        steering_angle,
        steps,
        reward,
        x,
        y,
        source[0],
        source[1],
        target[0],
        target[1],
        nearest[0],
        nearest[1],
        clothest_0[0],
        clothest_0[1],
        clothest_1[0],
        clothest_1[1],
        heading,
        best_dir,
        steering,
        predicted,
        target_distance_view,
        dist,
        speed,
        speed_ratio)

    return reward


def compute_distance_view(x, y, source_idx, waypoints, track_width):
    target_distance_view = 2
    target_idx = source_idx + target_distance_view

    while True:
        target = waypoints[target_idx % len(waypoints)]

        for i in range(source_idx + 1, target_idx):
            current = waypoints[i % len(waypoints)]
            dist, nearest = distance_point_to_line(current[0], current[1], x, y, target[0], target[1])
            if dist > track_width / 2:
                return target_distance_view, dist, nearest

        target_distance_view = target_distance_view + 1
        target_idx = source_idx + target_distance_view


def log(waypoints, closest_waypoints, track_width, steering_angle, steps, reward,
        vehicle_x, vehicle_y, vehicle_source_x, vehicle_source_y, vehicle_target_x,
        vehicle_target_y, vehicle_target_nearest_x, vehicle_target_nearest_y,
        vehicle_clothest_0_x, vehicle_clothest_0_y, vehicle_clothest_1_x, vehicle_clothest_1_y,
        vehicle_heading, vehicle_best_dir, vehicle_steering, vehicle_predicted,
        target_distance, target_distance_view, speed, speed_ratio):

    coord0 = waypoints[closest_waypoints[0]]
    coord1 = waypoints[closest_waypoints[1]]
    myradians = math.atan2(coord1[1] - coord0[1], coord1[0] - coord0[0])
    mydegrees = math.degrees(myradians)

    print("Waypoint0:{},"
          "X:{},"
          "Y:{},"
          "heading:{},"
          "trackwidth:{},"
          "steering_angle:{},"
          "steps:{},"
          "reward:{},"
          "vehicle_x:{},"
          "vehicle_y:{},"
          "vehicle_source_x:{},"
          "vehicle_source_y:{},"
          "vehicle_target_x:{},"
          "vehicle_target_y:{},"
          "vehicle_target_nearest_x:{},"
          "vehicle_target_nearest_y:{},"
          "vehicle_closest_0_x:{},"
          "vehicle_closest_0_y:{},"
          "vehicle_closest_1_x:{},"
          "vehicle_closest_1_y:{},"
          "vehicle_heading:{},"
          "vehicle_best_dir:{},"
          "vehicle_steering:{},"
          "vehicle_predicted:{},"
          "vehicle_target_distance_view:{},"
          "vehicle_target_distance:{},"
          "vehicle_speed:{},"
          "vehicle_speed_ratio:{},".format(
        closest_waypoints[0],
        coord0[0],
        coord0[1],
        mydegrees,
        track_width,
        steering_angle,
        steps,
        reward,
        vehicle_x,
        vehicle_y,
        vehicle_source_x,
        vehicle_source_y,
        vehicle_target_x,
        vehicle_target_y,
        vehicle_target_nearest_x,
        vehicle_target_nearest_y,
        vehicle_clothest_0_x,
        vehicle_clothest_0_y,
        vehicle_clothest_1_x,
        vehicle_clothest_1_y,
        vehicle_heading,
        vehicle_best_dir,
        vehicle_steering,
        vehicle_predicted,
        target_distance_view,
        target_distance,
        speed,
        speed_ratio))


# HELPER

def dist_to(x1, y1, x2, y2):
    return math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))


def angle_min_diff(x, y):
    arg = (y - x) % 360
    if arg < 0:
        arg = arg + 360
    if arg > 180:
        arg = arg - 360
    return -arg


def atan2_deg(x1, y1, x2, y2):
    rad = math.atan2(y2 - y1, x2 - x1)
    deg = math.degrees(rad)
    return nor(deg)


def nor(angle):
    angle = angle % 360
    if angle < 0:
        angle = angle + 360
    return angle


def vector_dot(v, w):
    x, y, z = v
    xx, yy, zz = w
    return x * xx + y * yy + z * zz


def vector_length(v):
    x, y, z = v
    return math.sqrt(x * x + y * y + z * z)


def vector(b, e):
    x, y, z = b
    xx, yy, zz = e
    return xx - x, yy - y, zz - z


def vector_unit(v):
    x, y, z = v
    mag = vector_length(v)
    return x / mag, y / mag, z / mag


def vector_distance(p0, p1):
    return vector_length(vector(p0, p1))


def vector_scale(v, sc):
    x, y, z = v
    return x * sc, y * sc, z * sc


def vector_add(v, w):
    x, y, z = v
    xx, yy, zz = w
    return x + xx, y + yy, z + zz


def distance_point_to_line(x, y, x1, y1, x2, y2):
    pnt = (x, y, 0)
    start = (x1, y1, 0)
    end = (x2, y2, 0)

    line_vec = vector(start, end)
    pnt_vec = vector(start, pnt)
    line_len = vector_length(line_vec)
    line_unit_vec = vector_unit(line_vec)
    pnt_vec_scaled = vector_scale(pnt_vec, 1.0 / line_len)
    t = vector_dot(line_unit_vec, pnt_vec_scaled)
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    nearest = vector_scale(line_vec, t)
    dist = vector_distance(nearest, pnt_vec)
    nearest = vector_add(nearest, start)
    return dist, nearest
