def reward_function(params):
    import math

    score_exc = 20
    score_max = 5

    progress = params['progress']
    steering_angle = params['steering_angle']
    steps = params['steps']
    track_width = params['track_width']
    heading = params['heading']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']

    x = params['x']
    y = params['y']
    source_idx = closest_waypoints[0]
    target_idx = source_idx + 2

    # algorithm to find the farest visible waypoints
    loop = True
    while loop:
        target = waypoints[target_idx % len(waypoints)]

        for i in range(source_idx + 1, target_idx):
            current = waypoints[i % len(waypoints)]
            if (dps(current[0], current[1], x, y, target[0], target[1]) >= math.hypot(track_width / 2,
                                                                                      track_width / 2)):
                loop = False
                break

        if loop:
            target_idx = target_idx + 1
        else:
            target_idx = target_idx - 3

    target = waypoints[target_idx % len(waypoints)]
    print("target: " + str(target))

    best_dir = nor(atan2_deg(x, y, target[0], target[1]))
    print("best dir: " + str(best_dir))

    heading = nor(heading)
    print("heading: " + str(heading))

    steering = nor(heading + steering_angle)
    print("steering: " + str(steering))

    predicted = nor(heading + 0.0 * steering_angle)
    print("predicted: " + str(predicted))

    angle_diff = math.fabs(angle_min_diff(predicted, best_dir))
    print("diff: " + str(angle_diff))

    # reward computing for best dir
    ratio = pow(float(1 - angle_diff / 180), 2)
    reward = round(score_max * ratio, 1)

    # reward computing for progress
    if progress == 100:
        reward += 5 * score_exc

    log(waypoints,
        closest_waypoints,
        track_width,
        steering_angle,
        steps,
        x, y,
        target[0], target[1],
        heading, best_dir,
        steering, predicted,
        reward)

    return reward


def dps(px, py, x1, y1, x2, y2):
    import math

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
    import math
    rad = math.atan2(y2 - y1, x2 - x1)
    deg = math.degrees(rad)
    return nor(deg)


def nor(angle):
    angle = angle % 360
    if angle < 0:
        angle = angle + 360
    return angle


def log(waypoints, closest_waypoints, track_width, steering_angle, steps,
        vehicle_x, vehicle_y, vehicle_target_x, vehicle_target_y, vehicle_heading,
        vehicle_best_dir, vehicle_steering, vehicle_predicted, reward):

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
          "vehicle_x:{},"
          "vehicle_y:{},"
          "vehicle_target_x:{},"
          "vehicle_target_y:{},"
          "vehicle_heading:{},"
          "vehicle_best_dir:{},"
          "vehicle_steering:{},"
          "vehicle_predicted:{},"
          "reward:{}".format(
            closest_waypoints[0],
            coord0[0],
            coord0[1],
            mydegrees,
            track_width,
            steering_angle,
            steps,
            vehicle_x,
            vehicle_y,
            vehicle_target_x,
            vehicle_target_y,
            vehicle_heading,
            vehicle_best_dir,
            vehicle_steering,
            vehicle_predicted,
            reward))
