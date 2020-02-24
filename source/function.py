import math


def reward_function(params):
    import math

    # parameters
    prediction_weight = 0.7
    waypoint_view_min = 10
    waypoint_view_max = 30
    speed_max = 2.7
    score_max_direction = 5
    score_max_race_complete = 20

    is_crashed = params['is_crashed']
    is_offtrack = params['is_offtrack']
    is_reversed = params['is_reversed']

    progress = params['progress']
    speed = params['speed']
    steering_angle = params['steering_angle']
    steps = params['steps']
    track_width = params['track_width']
    heading = params['heading']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']

    x = params['x']
    y = params['y']
    source_idx = closest_waypoints[0]

    speed_ratio = speed / speed_max

    # find target
    view_distance = compute_view_distance(x, y, source_idx, waypoints, waypoint_view_max, track_width)
    view_distance_speed_factor = 0.5 + (1 - speed_ratio) * 0.5
    target_distance = max(waypoint_view_min, round(min(view_distance, waypoint_view_max) * view_distance_speed_factor))
    target_idx = source_idx + target_distance

    target = waypoints[target_idx % len(waypoints)]

    # best dir
    best_dir = nor(atan2_deg(x, y, target[0], target[1]))
    heading = nor(heading)
    steering = nor(heading + steering_angle)

    prediction_ratio = (1 - prediction_weight) + speed_ratio * prediction_weight
    predicted = nor(heading + prediction_ratio * steering_angle)

    direction_diff = math.fabs(angle_min_diff(predicted, best_dir))
    direction_diff_ratio = pow(float(1 - direction_diff / 180), 2)

    # reward
    reward = round(score_max_direction * direction_diff_ratio, 1)

    if is_crashed or is_offtrack or is_reversed:
        reward = 0.0
    if progress == 100:
        reward += score_max_race_complete

    log(waypoints,
        closest_waypoints,
        track_width,
        steering_angle,
        steps,
        reward,
        x,
        y,
        target[0],
        target[1],
        heading,
        best_dir,
        steering,
        predicted,
        target_distance,
        view_distance,
        speed,
        speed_ratio)

    return reward


def compute_view_distance(x, y, source_idx, waypoints, waypoint_view_max, track_width):
    view_distance = 2
    target_idx = source_idx + view_distance

    while True:
        target = waypoints[target_idx % len(waypoints)]

        for i in range(source_idx + 1, target_idx):
            current = waypoints[i % len(waypoints)]
            if dps(current[0], current[1], x, y, target[0], target[1]) >= math.hypot(track_width / 2, track_width / 2):
                return view_distance

        view_distance = view_distance + 1
        target_idx = source_idx + view_distance


def dps(px, py, x1, y1, x2, y2):
    mag = math.hypot(x2 - x1, y2 - y1)

    if mag < 0.00000001:
        dst = 9999
        return dst

    u1 = (((px - x1) * (x2 - x1)) + ((py - y1) * (y2 - y1)))
    u = u1 / (mag * mag)

    if (u < 0.00001) or (u > 1):
        ix = math.hypot(x1 - px, y1 - py)
        iy = math.hypot(x2 - px, y2 - py)
        if ix > iy:
            dst = iy
        else:
            dst = ix
    else:
        ix = x1 + u * (x2 - x1)
        iy = y1 + u * (y2 - y1)
        dst = math.hypot(ix - px, iy - py)

    return dst


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


def log(waypoints, closest_waypoints, track_width, steering_angle, steps, reward,
        vehicle_x, vehicle_y, vehicle_target_x, vehicle_target_y, vehicle_heading,
        vehicle_best_dir, vehicle_steering, vehicle_predicted, target_distance,
        view_distance, speed, speed_ratio):

    import math
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
          "vehicle_target_x:{},"
          "vehicle_target_y:{},"
          "vehicle_heading:{},"
          "vehicle_best_dir:{},"
          "vehicle_steering:{},"
          "vehicle_predicted:{},"
          "vehicle_target_distance:{},"
          "vehicle_view_distance:{},"
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
            vehicle_target_x,
            vehicle_target_y,
            vehicle_heading,
            vehicle_best_dir,
            vehicle_steering,
            vehicle_predicted,
            target_distance,
            view_distance,
            speed,
            speed_ratio))
