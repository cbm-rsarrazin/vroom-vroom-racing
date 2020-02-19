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
        vehicle_x = float(commasplit[7].split(':')[1].strip())
        vehicle_y = float(commasplit[8].split(':')[1].strip())
        vehicle_target_x = float(commasplit[9].split(':')[1].strip())
        vehicle_target_y = float(commasplit[10].split(':')[1].strip())
        vehicle_heading = float(commasplit[11].split(':')[1].strip())
        vehicle_best_dir = float(commasplit[12].split(':')[1].strip())
        vehicle_steering = float(commasplit[13].split(':')[1].strip())
        vehicle_predicted = float(commasplit[14].split(':')[1].strip())
        reward = float(commasplit[15].split(':')[1].strip())

        coord = {'waypoint': waypoint, 'x': x, 'y': y, 'heading': heading, 'trackwidth': trackwidth,
                 'steering': steering, 'steps': steps,
                 'vehicle_x': vehicle_x, 'vehicle_y': vehicle_y,
                 'vehicle_target_x': vehicle_target_x, 'vehicle_target_y': vehicle_target_y,
                 'vehicle_heading': vehicle_heading, 'vehicle_best_dir': vehicle_best_dir,
                 'vehicle_predicted': vehicle_predicted, 'vehicle_steering': vehicle_steering,
                 'reward': reward}

        coords.append(coord)
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
red = (1, 0, 0)
green = (0, 1, 0)
blue = (0, 0, 1)
yellow = (1, 1, 0)

# vehicle position + heading
rewards = []
for i in range(len(coords)):
    coord = coords[i]

# coord = {'waypoint': waypoint, 'x': x, 'y': y, 'heading': heading, 'trackwidth': trackwidth,
#          'steering': steering, 'steps': steps,
#          'vehicle_x': vehicle_x, 'vehicle_y': vehicle_y,
#          'vehicle_target_x': vehicle_target_x, 'vehicle_target_y': vehicle_target_y,
#          'vehicle_heading': vehicle_heading, 'vehicle_best_dir': vehicle_best_dir,
#          'vehicle_predicted': vehicle_predicted, 'reward': reward}

    vehicle_x = coord['vehicle_x']
    vehicle_y = coord['vehicle_y']
    target_x = coord['vehicle_target_x']
    target_y = coord['vehicle_target_y']
    vehicle_heading = coord['vehicle_heading']
    vehicle_best_dir = coord['vehicle_best_dir']
    vehicle_steering = coord['vehicle_steering']
    vehicle_predicted = coord['vehicle_predicted']
    reward = coord['reward']

    vehicle_heading_point = get_point_from_angle(vehicle_x, vehicle_y, vehicle_heading, 0.5)
    vehicle_heading_x = vehicle_heading_point[0]
    vehicle_heading_y = vehicle_heading_point[1]

    vehicle_best_dir_point = get_point_from_angle(vehicle_x, vehicle_y, vehicle_best_dir, 0.5)
    vehicle_best_dir_x = vehicle_best_dir_point[0]
    vehicle_best_dir_y = vehicle_best_dir_point[1]

    vehicle_steering_point = get_point_from_angle(vehicle_x, vehicle_y, vehicle_steering, 0.5)
    vehicle_steering_x = vehicle_steering_point[0]
    vehicle_steering_y = vehicle_steering_point[1]

    vehicle_predicted_point = get_point_from_angle(vehicle_x, vehicle_y, vehicle_predicted, 0.5)
    vehicle_predicted_x = vehicle_predicted_point[0]
    vehicle_predicted_y = vehicle_predicted_point[1]

    if reward not in rewards:
        rewards.append(reward)

    if reward >= 100:
        plt.plot([vehicle_x, target_x], [vehicle_y, target_y], c=red)                           # target direction
        plt.plot([vehicle_x, vehicle_heading_x], [vehicle_y, vehicle_heading_y], c=green)       # heading direction
        plt.plot([vehicle_x, vehicle_predicted_x], [vehicle_y, vehicle_predicted_y], c=yellow)  # steering direction
        plt.plot([vehicle_x, vehicle_steering_x], [vehicle_y, vehicle_steering_y], c=blue)      # predicted direction

        plt.scatter(vehicle_x, vehicle_y, c=blue)           # vehicle position
        plt.scatter(target_x, target_y, c=blue)           # vehicle position

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
