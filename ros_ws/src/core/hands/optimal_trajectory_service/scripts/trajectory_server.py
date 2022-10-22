#!/usr/bin/env python

from __future__ import print_function

from optimal_trajectory_service.srv import Trajectory, TrajectoryResponse
from geometry_msgs.msg import Polygon, Point32
import rospy
import rospkg
import csv
import yaml

raceline = Polygon()
s = []
kappa = []
psi = []
acceleration = []
velocity = []


def get_trajectory(req):
    response = TrajectoryResponse()
    response.raceline.header.stamp = rospy.Time.now()
    response.raceline.header.frame_id = 'trajectory'
    response.raceline.polygon = raceline
    response.s = s
    response.kappa = kappa
    response.psi = psi
    response.acceleration = acceleration
    response.velocity = velocity
    return response


def loadPolygon():
    # path = os.path.join(os.path.expanduser('~'),'ARPG', 'artus','ros_ws','src','simulation','f1tenth-riders-quickstart','f1tenth_gym_ros','maps','SOCHI_OBS_2.yaml')
    # path = os.path.join(os.path.expanduser('~'), 'artus', 'ros_ws',
    #                    'src', 'simulation', 'optimal_trajectory_service', 'maps', 'SOCHI')
    map = rospy.get_param("/trajectory_server/map")
    rospack = rospkg.RosPack()
    path = rospack.get_path("optimal_trajectory_service") + "/maps" + "/" + map

    # load map metadata
    with open(path + '.yaml', 'r') as yaml_stream:
        try:
            map_metadata = yaml.safe_load(yaml_stream)
            map_resolution = map_metadata['resolution']
            origin = map_metadata['origin']
            origin_x = origin[0]
            origin_y = origin[1]
        except yaml.YAMLError as ex:
            print(ex)

    with open(path + ".csv", 'r') as csv_stream:
        reader = csv.reader(csv_stream, delimiter=";")
        next(reader)
        for row in reader:
            # each row contains the following
            # [0] s_m; [1] x_m; [2] y_m; [3] psi_rad; [4] kappa_radpm; [5] vx_mps; [6] ax_mps2
            centerline_point = Point32()
            centerline_point.x = float(row[1]) # * map_resolution + origin_x
            centerline_point.y = float(row[2]) # * map_resolution + origin_y
            centerline_point.z = 0.0
            raceline.points.append(centerline_point)

            s.append(float(row[0]))
            acceleration.append(float(row[6]))
            psi.append(float(row[3]))
            kappa.append(float(row[4]))
            velocity.append(float(row[5]))
            


def server():
    loadPolygon()
    rospy.init_node('trajectory_server')
    s = rospy.Service('get_trajectory', Trajectory, get_trajectory)
    print("Ready to provide trajectory.")
    rospy.spin()


if __name__ == "__main__":
    server()
