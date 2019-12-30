def reward_function(params):
    import math

    ratio_score_min = 0.5
    score_max = 5

    track_width = params['track_width']
    heading = params['heading']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']

    x = params['x']
    y = params['y']
    source_idx = closest_waypoints[0]  # closest before car
    target_idx = source_idx + 2

    # algorithm to find the farest visible waypoints
    loop = True
    while loop:
        if source_idx == target_idx:
            # all waypoints has been checked
            return -5

        target = waypoints[target_idx % len(waypoints)]

        for i in range(source_idx + 1, target_idx):
            current = waypoints[i % len(waypoints)]
            print(current)
            if (dps(current[0], current[1], x, y, target[0], target[1]) >= math.hypot(track_width / 2,
                                                                                      track_width / 2)):
                loop = False
                break

        if loop:
            target_idx = target_idx + 1
        else:
            target_idx = target_idx - 1
            # then exit the loop

    target = waypoints[target_idx % len(waypoints)]

    # angle from car to the farest visible waypoint
    best_dir = atan2_deg(x, y, target[0], target[1])
    # angle from first closest waypoint to second clothest waypoint
    closest_waypoints_dir = atan2_deg(waypoints[closest_waypoints[0]][0], waypoints[closest_waypoints[0]][1],
                                      waypoints[closest_waypoints[1]][0], waypoints[closest_waypoints[1]][1])

    print("target: " + str(target))
    print("best dir: " + str(best_dir))
    print("waypoints dir: " + str(closest_waypoints_dir))

    # compute average angle between best and current waypoints
    avg_dir = math.atan2((math.sin(math.radians(closest_waypoints_dir)) + math.sin(math.radians(best_dir))) / 2,
                         (math.cos(math.radians(closest_waypoints_dir)) + math.cos(math.radians(best_dir))) / 2)
    avg_dir = math.degrees(avg_dir)
    avg_dir = nor(avg_dir)
    print("avg dir: " + str(avg_dir))

    heading = nor(heading)
    print("heading: " + str(heading))

    # compute angle diff
    angle_diff = angle_min_diff(heading, avg_dir)
    print("diff: " + str(angle_diff))

    # score computing
    score = round(float(1 - math.fabs(angle_diff) / 360), 1)
    if score < ratio_score_min:
        return -score_max
    return score_max * pow(score, 2)


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
