import math

import numpy as np
import scipy.interpolate as si
from scipy.spatial import distance

bezier_from_waypoint = 0
nb_waypoint_used = 30
nb_point_best_race = 100
steps_gap = 100
steps_total = 457

speed_max = 4
speed_granularity = 3

score_max_step = 15
score_max_speed = 15
score_max_distance = 15
score_max_direction = 5
score_max_complete = 100


def reward_function(params):
    x = params['x']
    y = params['y']
    heading = params['heading']
    waypoints = params['waypoints']
    track_width = params['track_width']
    speed = params['speed']
    progress = params['progress']
    steps = params['steps']
    is_offtrack = params['is_offtrack']

    reward = 1e-3

    # distance reward
    best_race = get_best_race(waypoints, bezier_from_waypoint, nb_waypoint_used, nb_point_best_race)
    nearest_index, nearest_interval_indexes = get_nearest_points(best_race, x, y)
    nearest1 = best_race[nearest_interval_indexes[0]]
    nearest2 = best_race[nearest_interval_indexes[1]]
    dist = distance_to_line(x, y, nearest1, nearest2)
    dist_ratio = 0.0
    if dist < track_width:
        dist_ratio = (track_width - dist) / track_width
    reward += dist_ratio * score_max_distance

    # speed reward
    best_speed = get_best_speed(best_race)
    current_best_speed = best_speed[nearest_index]
    current_speed = speed / speed_max
    speed_ratio = 1.0 - abs(current_speed - current_best_speed)
    reward += speed_ratio * score_max_speed

    # direction reward
    best_dir = nor(atan2_deg(best_race[nearest_interval_indexes[0]][0], best_race[nearest_interval_indexes[0]][1],
                             best_race[nearest_interval_indexes[1]][0], best_race[nearest_interval_indexes[1]][1]))
    heading = nor(heading)
    direction_diff = abs(angle_min_diff(heading, best_dir))
    direction_diff_ratio = 0.0
    if direction_diff <= 30:
        direction_diff_ratio = pow(float(1 - direction_diff / 180), 2)
    reward += direction_diff_ratio * score_max_direction

    # other reward
    if is_offtrack:
        reward = 1e-3
    elif progress == 100:
        reward += score_max_complete
    elif steps % steps_gap == 0 and progress > (steps / steps_total) * 100:
        reward += score_max_step

    return reward


def get_best_speed(best_race):
    angle_diff = []
    last_angle = atan2_deg(best_race[0][0], best_race[0][1], best_race[1][0], best_race[1][1])
    for i in range(1, len(best_race) + 1):
        p1_x = best_race[i % len(best_race)][0]
        p1_y = best_race[i % len(best_race)][1]
        p2_x = best_race[(i + 1) % len(best_race)][0]
        p2_y = best_race[(i + 1) % len(best_race)][1]
        angle = atan2_deg(p1_x, p1_y, p2_x, p2_y)
        diff = abs(angle_min_diff(last_angle, angle))
        angle_diff.append(diff)
        last_angle = angle

    max_diff = max(angle_diff)
    best_speed = list(map(lambda diff: 1 - diff/max_diff, angle_diff))
    best_speed = list(map(lambda diff: round(diff * speed_granularity) / speed_granularity, best_speed))

    return best_speed


def get_best_race(waypoints, loop_from, nb_waypoint, nb_point):
    waypoint_x = []
    waypoint_y = []
    gap = int(len(waypoints) / nb_waypoint)
    for i in range(loop_from, len(waypoints) + loop_from):
        if i % gap == 0:
            waypoint_x.append(waypoints[i % len(waypoints)][0])
            waypoint_y.append(waypoints[i % len(waypoints)][1])

    k = 3
    knot_space = range(len(waypoint_x))
    knots = si.InterpolatedUnivariateSpline(knot_space, knot_space, k=k).get_knots()
    knots_full = np.concatenate(([knots[0]] * k, knots, [knots[-1]] * k))

    tckX = knots_full, waypoint_x, k
    tckY = knots_full, waypoint_y, k

    splineX = si.UnivariateSpline._from_tck(tckX)
    splineY = si.UnivariateSpline._from_tck(tckY)

    tP = np.linspace(knot_space[0], knot_space[-1], nb_point)
    xP = splineX(tP)
    yP = splineY(tP)

    best_race = []
    for a, b in zip(xP, yP):
        best_race.append([a, b])

    return best_race


def get_nearest_points(best_race, x, y):
    nearest_index = distance.cdist([(x, y)], best_race).argmin()

    nearest_prev_index = (nearest_index - 1) % len(best_race)
    nearest_next_index = (nearest_index + 1) % len(best_race)

    nearest_prev = best_race[nearest_prev_index]
    nearest_next = best_race[nearest_next_index]

    nearest_prev_dist = math.sqrt(((nearest_prev[0]-x)**2)+((nearest_prev[1]-y)**2))
    nearest_next_dist = math.sqrt(((nearest_next[0]-x)**2)+((nearest_next[1]-y)**2))

    if nearest_prev_dist < nearest_next_dist:
        return nearest_index, [nearest_prev_index, nearest_index]
    else:
        return nearest_index, [nearest_index, nearest_next_index]


def atan2_deg(x1, y1, x2, y2):
    rad = math.atan2(y2 - y1, x2 - x1)
    deg = math.degrees(rad)
    return nor(deg)


def nor(angle):
    angle = angle % 360
    if angle < 0:
        angle = angle + 360
    return angle


def angle_min_diff(x, y):
    arg = (y - x) % 360
    if arg < 0:
        arg = arg + 360
    if arg > 180:
        arg = arg - 360
    return -arg


def distance_to_line(x, y, p1, p2):
    np1 = np.array(p1)
    np2 = np.array(p2)
    np3 = np.array([x, y])
    return abs(np.cross(np2 - np1, np3 - np1) / np.linalg.norm(np2 - np1))
