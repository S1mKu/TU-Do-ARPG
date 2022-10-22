#!/usr/bin/env python3.8

import platform
import subprocess
import rospy
import csv
import json
import os
from centerline_service.srv import Centerline
from std_msgs.msg import String

def get_centerline_response():
    rospy.wait_for_service('get_centerline')
    try:
        get_centerline = rospy.ServiceProxy('get_centerline', Centerline)
        resp1 = get_centerline()
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def start_node():
    #Init node
    rospy.init_node('calculate_global', anonymous=True)

    # Load MPC parameter from json file
    file_to_open = os.path.join(os.path.dirname(__file__), '', 'trajectory_parameter.json')
    with open(file_to_open, 'r') as f:
        global parameter
        parameter = json.load(f)

    centerline_response = get_centerline_response()
    print("Received Centerline Response")
    centerline = centerline_response.centerline
    points = centerline.polygon.points
    widths = centerline_response.width
    min_width = min(widths)

    csv_file_to_open = os.path.join(os.path.dirname(__file__), 'global_racetrajectory_optimization/inputs/tracks', 'centerline_file.csv')
    with open(csv_file_to_open, mode='w') as centerline_file:
        centerline_writer = csv.writer(centerline_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_NONNUMERIC)

        for point in reversed(centerline.polygon.points):
            # print(point.x)
            centerline_writer.writerow([point.x, point.y, min_width/2, min_width/2])

    print("Wrote csv file for input")
    python_script_to_run = os.path.join(os.path.dirname(__file__), 'global_racetrajectory_optimization', 'main_globaltraj.py')
    # os.system(python_script_to_run)
    # subprocess.Popen(python_script_to_run, shell=True)

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass