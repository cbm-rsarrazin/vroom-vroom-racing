import math

import matplotlib.pyplot as plt
import numpy as np
import scipy.interpolate as si
from scipy.spatial import distance
from shapely.geometry import Point, Polygon


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

    return best_speed


def get_best_race(waypoints, nb_waypoint, nb_point):
    waypoint_x = []
    waypoint_y = []
    gap = int(len(waypoints) / nb_waypoint)
    for i in range(0, len(waypoints)):
        if i % gap == 0:
            waypoint_x.append(waypoints[i][0])
            waypoint_y.append(waypoints[i][1])

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


if __name__ == "__main__":
    # ----------------------- Params

    speed_max = 4
    nb_waypoint_used = 10
    nb_point_best_race = 50

    score_max_speed = 10
    score_max_distance = 15
    score_max_direction = 5
    score_max_complete = 50

    waypoints = np.load('tracks/reInvent2019_wide.npy')
    x = 6
    y = 1
    heading = 0
    track_width = 1
    speed = 4
    progress = 50
    is_offtrack = False
    steps = 1

    # ----------------------- Function

    reward = 1e-3

    # distance reward
    best_race = get_best_race(waypoints, nb_waypoint_used, nb_point_best_race)
    nearest_index, nearest_interval_indexes = get_nearest_points(best_race, x, y)
    poly_race = Polygon(best_race)
    dist = poly_race.distance(Point(x, y))
    dist_ratio = 0
    if dist < track_width:
        dist_ratio = (track_width - dist) / track_width
    reward += dist_ratio * score_max_distance
    print("distance: " + str(dist_ratio))

    # speed reward
    best_speed = get_best_speed(best_race)
    current_best_speed = best_speed[nearest_index]
    current_speed = speed / speed_max
    speed_ratio = 1 - abs(current_speed - current_best_speed)
    reward += speed_ratio * score_max_speed
    print("speed: " + str(speed_ratio))

    # direction reward
    best_dir = nor(atan2_deg(best_race[nearest_interval_indexes[0]][0], best_race[nearest_interval_indexes[0]][1],
                             best_race[nearest_interval_indexes[1]][0], best_race[nearest_interval_indexes[1]][1]))
    heading = nor(heading)
    direction_diff = abs(angle_min_diff(heading, best_dir))
    direction_diff_ratio = 0
    if direction_diff <= 30:
        direction_diff_ratio = pow(float(1 - direction_diff / 180), 2)
    reward += direction_diff_ratio * score_max_direction
    print("direction: " + str(direction_diff_ratio))

    # ----------------------- Plot

    best_x = []
    best_y = []
    for i in range(0, len(best_race)):
        best_x.append(best_race[i][0])
        best_y.append(best_race[i][1])

    center_line = waypoints[:, 0:2]
    inner_border = waypoints[:, 2:4]
    outer_border = waypoints[:, 4:6]

    cx = center_line[:, 0]
    cy = center_line[:, 1]

    ix = inner_border[:, 0]
    iy = inner_border[:, 1]

    ox = outer_border[:, 0]
    oy = outer_border[:, 1]

    plt.plot(cx, cy, color='gray')  # center line
    plt.plot(ix, iy, color='lightgray')  # inner line
    plt.plot(ox, oy, color='lightgray')  # outer line

    for i in range(len(best_speed)):
        ratio = best_speed[i]
        plt.scatter(best_x[i], best_y[i], color=(math.sqrt(1.0 - ratio), 0, ratio**2))

    # plt.scatter(best_x[0], best_y[0], color='green')  # first node
    # plt.scatter(best_x[len(best_x) - 1], best_y[len(best_y) - 1], color='red')  # last node
    # plt.scatter(best_race[nearest_interval_indexes[0]][0], best_race[nearest_interval_indexes[0]][1], color='green')
    # plt.scatter(best_race[nearest_interval_indexes[1]][0], best_race[nearest_interval_indexes[1]][1], color='green')

    plt.plot([x, x + math.cos(math.radians(heading))], [y, y + math.sin(math.radians(heading))], color='black')
    plt.scatter(x, y, color='black')

    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()
