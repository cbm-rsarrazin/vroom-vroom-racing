import argparse
import datetime
import math

import boto3
import matplotlib.patches as mpatches
import matplotlib.path as mpath
import matplotlib.pyplot as plt

# args
parser = argparse.ArgumentParser(description='Generate a Deepracer Heatmap')
parser.add_argument('--profile', required=True, help='aws credentials profile')
parser.add_argument('--logstreamname', required=True, help='e.g. sim-nd2x8c3ph1d3/2019-06-06T14-39-31.940Z_3f5ddea9-6555-44d0-b1c7-ceb4b054f884/SimulationApplicationLogs')

args = parser.parse_args()

# constants
starttimeepoch = int(datetime.datetime(2018, 1, 30, 18, 0, 0).timestamp()) * 1000
endtimeepoch = int(datetime.datetime(2030, 1, 30, 18, 0, 0).timestamp()) * 1000
loggroupname = '/aws/robomaker/SimulationJobs'
logstreamname = args.logstreamname
profile = args.profile
region = 'us-east-1'


def get_heatmap_data(loggroupname, logstreamname, starttimeepoch, endtimeepoch):
    filterPattern = 'SIM_TRACE_LOG'

    nextToken = ''
    count = 0
    events = []
    while nextToken is not None:
        if nextToken == '':
            response = logs_client.filter_log_events(
                logGroupName=loggroupname,
                logStreamNames=[logstreamname],
                startTime=starttimeepoch,
                endTime=endtimeepoch,
                filterPattern=filterPattern,
                limit=5000
            )
        else:
            response = logs_client.filter_log_events(
                logGroupName=loggroupname,
                logStreamNames=[logstreamname],
                startTime=starttimeepoch,
                endTime=endtimeepoch,
                filterPattern=filterPattern,
                nextToken=nextToken,
                limit=5000
            )
        events += response['events']
        count += len(response['events'])
        print(count)

        # while
        if 'nextToken' not in response.keys():
            print('Data Collected...')
            break
        else:
            nextToken = response['nextToken']
            # break # unhash out for testing
        # print (nextToken)

    # print (len(events))
    xs = []
    ys = []
    print('Parsing Data...')
    for event in events:
        split = event['message'].split(':')[1].split(',')
        x = float(split[2])
        y = float(split[3])
        xs.append(x)
        ys.append(y)
        # print("X: {}, Y: {}".format(x,y))

    # print(len(xs))
    # print(len(ys))
    return xs, ys


def get_string_path_data(loggroupname, logstreamname, starttimeepoch, endtimeepoch):
    filterPattern='Waypoint0'
    nextToken = ''
    count = 0
    events = []
    while nextToken is not None:
        if nextToken == '':
            response = logs_client.filter_log_events(
                logGroupName=loggroupname,
                logStreamNames=[logstreamname],
                startTime=starttimeepoch,
                endTime=endtimeepoch,
                filterPattern=filterPattern,
                limit=5000
            )
        else:
            response = logs_client.filter_log_events(
                logGroupName=loggroupname,
                logStreamNames=[logstreamname],
                startTime=starttimeepoch,
                endTime=endtimeepoch,
                filterPattern=filterPattern,
                nextToken=nextToken,
                limit=5000
            )
        events += response['events']
        count += len(response['events'])
        print(count)

        # while
        if 'nextToken' not in response.keys():
            print('End')
            break
        else:
            nextToken = response['nextToken']
            # break # unhash out for testing
        # print (nextToken)

    print(len(events))

    coords = []
    print('Parsing Waypoint Data...')
    for event in events:
        commasplit = event['message'].split(',')

        waypoint = int(commasplit[0].split(':')[1].strip())
        x = float(commasplit[1].split(':')[1].strip())
        y = float(commasplit[2].split(':')[1].strip())
        heading = float(commasplit[3].split(':')[1].strip())
        trackwidth = float(commasplit[4].split(':')[1].strip())
        steering = float(commasplit[5].split(':')[1].strip())
        steps = float(commasplit[6].split(':')[1].strip())
        reward = float(commasplit[7].split(':')[1].strip())
        vehicle_x = float(commasplit[8].split(':')[1].strip())
        vehicle_y = float(commasplit[9].split(':')[1].strip())
        vehicle_source_x = float(commasplit[10].split(':')[1].strip())
        vehicle_source_y = float(commasplit[11].split(':')[1].strip())
        vehicle_target_x = float(commasplit[12].split(':')[1].strip())
        vehicle_target_y = float(commasplit[13].split(':')[1].strip())
        vehicle_target_nearest_x = float(commasplit[14].split(':')[1].strip())
        vehicle_target_nearest_y = float(commasplit[15].split(':')[1].strip())
        vehicle_heading = float(commasplit[16].split(':')[1].strip())
        vehicle_best_dir = float(commasplit[17].split(':')[1].strip())
        vehicle_steering = float(commasplit[18].split(':')[1].strip())
        vehicle_predicted = float(commasplit[19].split(':')[1].strip())
        vehicle_target_distance_view = float(commasplit[20].split(':')[1].strip())
        vehicle_target_distance = float(commasplit[21].split(':')[1].strip())
        vehicle_speed = float(commasplit[22].split(':')[1].strip())
        vehicle_speed_ratio = float(commasplit[23].split(':')[1].strip())

        coords.append({'waypoint': waypoint,
                       'x': x,
                       'y': y,
                       'heading': heading,
                       'trackwidth': trackwidth,
                       'steering': steering,
                       'steps': steps,
                       'reward': reward,
                       'vehicle_x': vehicle_x,
                       'vehicle_y': vehicle_y,
                       'vehicle_source_x': vehicle_source_x,
                       'vehicle_source_y': vehicle_source_y,
                       'vehicle_target_x': vehicle_target_x,
                       'vehicle_target_y': vehicle_target_y,
                       'vehicle_target_nearest_x': vehicle_target_nearest_x,
                       'vehicle_target_nearest_y': vehicle_target_nearest_y,
                       'vehicle_heading': vehicle_heading,
                       'vehicle_best_dir': vehicle_best_dir,
                       'vehicle_steering': vehicle_steering,
                       'vehicle_predicted': vehicle_predicted,
                       'vehicle_target_distance_view': vehicle_target_distance_view,
                       'vehicle_target_distance': vehicle_target_distance,
                       'vehicle_speed': vehicle_speed,
                       'vehicle_speed_ratio': vehicle_speed_ratio})
        # print("X: {}, Y: {}".format(x,y))

    # print(len(coords))
    uniquewaypoints = list({v['waypoint']: v for v in coords}.values())  # get unique items in list of dicts
    print("Unique Waypoints:{}".format(len(uniquewaypoints)))
    uniquewaypoints = sorted(uniquewaypoints, key=lambda i: i['waypoint'])

    # center_string_path_data
    center_string_path_data = []
    firstwaypoint = True
    for waypoint in uniquewaypoints:
        x = waypoint['x']
        y = waypoint['y']
        if firstwaypoint:
            center_string_path_data.append((mpath.Path.MOVETO, (x, y)))
        else:
            center_string_path_data.append((mpath.Path.LINETO, (x, y)))
        firstwaypoint = False
    center_string_path_data.append((mpath.Path.CLOSEPOLY, (0, 0)))  # close polygon

    # inside_string_path_data
    inside_string_path_data = []
    firstwaypoint = True
    for waypoint in uniquewaypoints:
        x = waypoint['x'] + (waypoint['trackwidth']/2) * math.cos(math.radians(waypoint['heading']+90))
        y = waypoint['y'] + (waypoint['trackwidth']/2) * math.sin(math.radians(waypoint['heading']+90))
        if firstwaypoint:
            inside_string_path_data.append((mpath.Path.MOVETO, (x, y)))
        else:
            inside_string_path_data.append((mpath.Path.LINETO, (x, y)))
        firstwaypoint = False
    inside_string_path_data.append((mpath.Path.CLOSEPOLY, (0, 0)))  # close polygon

    # outside_string_path_data
    outside_string_path_data = []
    firstwaypoint = True
    for waypoint in uniquewaypoints:
        x = waypoint['x'] + (waypoint['trackwidth']/2) * math.cos(math.radians(waypoint['heading']-90))
        y = waypoint['y'] + (waypoint['trackwidth']/2) * math.sin(math.radians(waypoint['heading']-90))
        if firstwaypoint:
            outside_string_path_data.append((mpath.Path.MOVETO, (x, y)))
        else:
            outside_string_path_data.append((mpath.Path.LINETO, (x, y)))
        firstwaypoint = False
    outside_string_path_data.append((mpath.Path.CLOSEPOLY, (0, 0)))  # close polygon

    return center_string_path_data, inside_string_path_data, outside_string_path_data, coords


def get_point_from_angle(x, y, angle_deg, dist):
    angle_rad = math.radians(angle_deg)
    return x + (dist * math.cos(angle_rad)), y + (dist * math.sin(angle_rad))


def round_2(val):
    return round(val * 100) / 100

# MAIN


session = boto3.Session(profile_name=profile, region_name=region)

logs_client = session.client('logs')

# collect heatmap data
heatmap_data = get_heatmap_data(loggroupname, logstreamname, starttimeepoch, endtimeepoch)
xs = heatmap_data[0]
ys = heatmap_data[1]

fig, ax = plt.subplots(figsize=(8, 6))

print('Loading Heatmap Data...')
hist = plt.hist2d(xs, ys, bins=(250, 125), normed=False, cmap='hot')
plt.xlabel('x axis')
plt.ylabel('y axis')

# collect track limits data
string_path_data = get_string_path_data(loggroupname, logstreamname, starttimeepoch, endtimeepoch)

coords = list(string_path_data[3])
uniquewaypoints = list({v['waypoint']: v for v in coords}.values())


# repartition
# for i in range(len(coords)):
#     if i % 1000 == 0:
#         coord = coords[i]
#
#         vehicle_x = coord['vehicle_x']
#         vehicle_y = coord['vehicle_y']
#
#         plt.scatter(vehicle_x, vehicle_y, color='blue', alpha=0.01)


# direction
# for i in range(len(coords)):
#     coord = coords[i]
#
#     reward = coord['reward']
#     vehicle_x = coord['vehicle_x']
#     vehicle_y = coord['vehicle_y']
#     target_x = coord['vehicle_target_x']
#     target_y = coord['vehicle_target_y']
#     vehicle_target_distance = coord['vehicle_target_distance']
#     vehicle_view_distance = coord['vehicle_view_distance']
#     vehicle_speed = coord['vehicle_speed']
#     vehicle_speed_ratio = coord['vehicle_speed_ratio']
#     vehicle_heading = coord['vehicle_heading']
#     vehicle_best_dir = coord['vehicle_best_dir']
#     vehicle_steering = coord['vehicle_steering']
#     vehicle_predicted = coord['vehicle_predicted']
#
#     if i % 10000 == 0:
#         vehicle_heading_point = get_point_from_angle(vehicle_x, vehicle_y, vehicle_heading, 0.5)
#         vehicle_heading_x = vehicle_heading_point[0]
#         vehicle_heading_y = vehicle_heading_point[1]
#
#         vehicle_best_dir_point = get_point_from_angle(vehicle_x, vehicle_y, vehicle_best_dir, 0.5)
#         vehicle_best_dir_x = vehicle_best_dir_point[0]
#         vehicle_best_dir_y = vehicle_best_dir_point[1]
#
#         vehicle_steering_point = get_point_from_angle(vehicle_x, vehicle_y, vehicle_steering, 0.5)
#         vehicle_steering_x = vehicle_steering_point[0]
#         vehicle_steering_y = vehicle_steering_point[1]
#
#         vehicle_predicted_point = get_point_from_angle(vehicle_x, vehicle_y, vehicle_predicted, 0.5)
#         vehicle_predicted_x = vehicle_predicted_point[0]
#         vehicle_predicted_y = vehicle_predicted_point[1]
#
#         plt.plot([vehicle_x, target_x], [vehicle_y, target_y], color='red')
#         plt.plot([vehicle_x, vehicle_heading_x], [vehicle_y, vehicle_heading_y], color='green')
#         plt.plot([vehicle_x, vehicle_predicted_x], [vehicle_y, vehicle_predicted_y], color='yellow')
#         plt.plot([vehicle_x, vehicle_steering_x], [vehicle_y, vehicle_steering_y], color='blue')
#
#         plt.scatter(vehicle_x, vehicle_y, color='blue')       # vehicle position
#         plt.scatter(target_x, target_y, color='red')          # target position


# speed to target
for i in range(len(coords)):
    coord = coords[i]

    reward = coord['reward']
    vehicle_x = coord['vehicle_x']
    vehicle_y = coord['vehicle_y']
    target_x = coord['vehicle_target_x']
    target_y = coord['vehicle_target_y']
    nearest_x = coord['vehicle_target_nearest_x']
    nearest_y = coord['vehicle_target_nearest_y']
    vehicle_speed_ratio = coord['vehicle_speed_ratio']
    vehicle_best_dir = coord['vehicle_best_dir']

    if reward > 0 and i % 2000 == 0:
        dst = vehicle_speed_ratio * math.sqrt(math.pow(target_x - vehicle_x, 2) + math.pow(target_y - vehicle_y, 2))

        vehicle_best_dir_point = get_point_from_angle(vehicle_x, vehicle_y, vehicle_best_dir, dst)
        vehicle_best_dir_x = vehicle_best_dir_point[0]
        vehicle_best_dir_y = vehicle_best_dir_point[1]

        plt.plot([vehicle_x, target_x], [vehicle_y, target_y], color='red')
        plt.plot([vehicle_x, vehicle_best_dir_x], [vehicle_y, vehicle_best_dir_y], color='white')

        plt.scatter(vehicle_x, vehicle_y, color='blue')       # vehicle position
        plt.scatter(target_x, target_y, color='red')          # target position
        plt.scatter(nearest_x, nearest_y, color='green')      # nearest position


# print
print('\n')

rewards = []
avg_steps = 0
min_steps = 99999999
max_steps = -1
avg_speed = 0
avg_distance_view = 0
min_distance_view = 99999999
max_distance_view = -1
avg_target_distance = 0
avg_track_width = 0

for i in range(len(coords)):
    coord = coords[i]

    reward = coord['reward']
    steps = coord['steps']
    track_width = coord['trackwidth']
    vehicle_target_distance_view = coord['vehicle_target_distance_view']
    vehicle_target_distance = coord['vehicle_target_distance']
    vehicle_speed = coord['vehicle_speed']
    vehicle_speed_ratio = coord['vehicle_speed_ratio']
    vehicle_x = round_2(coord['vehicle_x'])
    vehicle_y = round_2(coord['vehicle_y'])
    target_x = round_2(coord['vehicle_target_x'])
    target_y = round_2(coord['vehicle_target_y'])
    nearest_x = round_2(coord['vehicle_target_nearest_x'])
    nearest_y = round_2(coord['vehicle_target_nearest_y'])

    if reward not in rewards:
        rewards.append(reward)

    avg_steps += steps
    min_steps = min(min_steps, steps)
    max_steps = max(max_steps, steps)
    avg_track_width += track_width
    avg_speed += vehicle_speed
    avg_target_distance += vehicle_target_distance
    avg_distance_view += vehicle_target_distance_view
    min_distance_view = min(min_distance_view, vehicle_target_distance_view)
    max_distance_view = max(max_distance_view, vehicle_target_distance_view)

    print("- vehicle:" + str((vehicle_x, vehicle_y)) +
          ", target:" + str((target_x, target_y)) +
          ", nearest:" + str((nearest_x, nearest_y)) +
          ", speed_ratio:" + str(vehicle_speed_ratio) +
          ", view_distance:" + str(vehicle_target_distance_view) +
          ", target_distance:" + str(vehicle_target_distance) +
          ", steps:" + str(steps) +
          ", reward:" + str(reward))

avg_steps /= len(coords)
avg_track_width /= len(coords)
avg_speed /= len(coords)
avg_distance_view /= len(coords)
avg_target_distance /= len(coords)

print('\n')
print('avg_step: ' + str(avg_steps))
print('min_step: ' + str(min_steps))
print('max_step: ' + str(max_steps))
print('avg_track_width: ' + str(avg_track_width))
print('avg_speed: ' + str(avg_speed))
print('avg_target_distance: ' + str(avg_target_distance))
print('avg_view_distance: ' + str(avg_distance_view))
print('min_view_distance: ' + str(min_distance_view))
print('max_view_distance: ' + str(max_distance_view))
print('rewards: ' + str(sorted(rewards)))
print('coords: ' + str(len(coords)))


# center
codes, verts = zip(*string_path_data[0])
string_path = mpath.Path(verts, codes)
patch = mpatches.PathPatch(string_path, edgecolor='white', facecolor="none", lw=1, ls='--')
ax.add_patch(patch)

# inside
codes, verts = zip(*string_path_data[1])
string_path = mpath.Path(verts, codes)
patch = mpatches.PathPatch(string_path, edgecolor='white', facecolor="none", lw=1)
ax.add_patch(patch)

# outside
codes, verts = zip(*string_path_data[2])
string_path = mpath.Path(verts, codes)
patch = mpatches.PathPatch(string_path, edgecolor='white', facecolor="none", lw=1)
ax.add_patch(patch)


print('Displaying Heatmap')
plt.show()
print('Finished')
