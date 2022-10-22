#!/usr/bin/env python

from __future__ import print_function

from centerline_service.srv import Centerline, CenterlineResponse
from geometry_msgs.msg import Polygon, Point32
import rospy
import rospkg
import csv
import yaml


centerline = {
    "centerline" : Polygon(),
    "roadside_inner" : Polygon(),
    "roadside_outer" : Polygon(),
    "width" : [],
    "velocity" : [],
}

true_centerline = {
    "centerline" : Polygon(),
    "roadside_inner" : Polygon(),
    "roadside_outer" : Polygon(),
    "width" : [],
    "velocity" : [],
}



def get_centerline(req):
    response = CenterlineResponse()
    response.centerline.header.stamp = rospy.Time.now()
    response.centerline.header.frame_id = 'centerline'
    response.centerline.polygon = centerline["centerline"]
    response.roadside_inner.polygon = centerline["roadside_inner"]
    response.roadside_outer.polygon = centerline["roadside_outer"]
    response.width = centerline["width"]
    response.velocity = centerline["velocity"]
    return response

def get_true_centerline(req):
    response = CenterlineResponse()
    response.centerline.header.stamp = rospy.Time.now()
    response.centerline.header.frame_id = 'centerline'
    response.centerline.polygon = true_centerline["centerline"]
    response.roadside_inner.polygon = true_centerline["roadside_inner"]
    response.roadside_outer.polygon = true_centerline["roadside_outer"]
    response.width = true_centerline["width"]
    response.velocity = true_centerline["velocity"]
    return response

def init():
    rospack = rospkg.RosPack()
    map_centerline = rospy.get_param("/centerline_server/map")
    map_true_centerline = rospy.get_param("/centerline_server/map_centerline")

    # load centerline which is used for all services
    path_centerline = rospack.get_path("centerline_service") + "/maps" + "/" + map_centerline
    c, ri, ro, w, v = loadPolygon(path_centerline, load_yml=False)
    centerline["centerline"] = c
    centerline["roadside_inner"] = ri
    centerline["roadside_outer"] = ro
    centerline["width"] = w
    centerline["velocity"] = v

    # load special centerline with true centerline without optimization
    path_true_centerline = rospack.get_path("centerline_service") + "/maps" + "/" + map_true_centerline
    c, ri, ro, w, v = loadPolygon(path_true_centerline, load_yml=False)
    true_centerline["centerline"] = c
    true_centerline["roadside_inner"] = ri
    true_centerline["roadside_outer"] = ro
    true_centerline["width"] = w
    true_centerline["velocity"] = v




def loadPolygon(map_path, load_yml=True):
    # path = os.path.join(os.path.expanduser('~'),'ARPG', 'artus','ros_ws','src','simulation','f1tenth-riders-quickstart','f1tenth_gym_ros','maps','SOCHI_OBS_2.yaml')
    # path = os.path.join(os.path.expanduser('~'), 'artus', 'ros_ws',
    #                    'src', 'simulation', 'centerline_service', 'maps', 'SOCHI')
    if load_yml:
        # load map metadata
        with open(map_path + '.yaml', 'r') as yaml_stream:
            try:
                map_metadata = yaml.safe_load(yaml_stream)
                map_resolution = map_metadata['resolution']
                origin = map_metadata['origin']
                origin_x = origin[0]
                origin_y = origin[1]
            except yaml.YAMLError as ex:
                print(ex)
    else:
        map_resolution = 1
        origin_x = 0
        origin_y = 0

    centerline = Polygon()
    roadside_inner = Polygon()
    roadside_outer = Polygon()
    width = []
    velocity = []

    with open(map_path + ".csv", 'r') as csv_stream:
        reader = csv.reader(csv_stream)
        next(reader)
        for row in reader:
            # each row contains the following
            # [0] center_x, [1] center_y, [2] inner_x, [3] inner_y, [4] outer_x, [5] outer_y, [6] dist_inner, [7] dist_outer
            centerline_point = Point32()
            centerline_point.x = float(row[0]) * map_resolution + origin_x
            centerline_point.y = float(row[1]) * map_resolution + origin_y
            centerline_point.z = 0.0
            centerline.points.append(centerline_point)

            roadside_inner_point = Point32()
            roadside_inner_point.x = float(row[2]) * map_resolution + origin_x
            roadside_inner_point.y = float(row[3]) * map_resolution + origin_y
            roadside_inner_point.z = 0.0
            roadside_inner.points.append(roadside_inner_point)

            roadside_outer_point = Point32()
            roadside_outer_point.x = float(row[4]) * map_resolution + origin_x
            roadside_outer_point.y = float(row[5]) * map_resolution + origin_y
            roadside_outer_point.z = 0.0
            roadside_outer.points.append(roadside_outer_point)

            width.append(min(float(row[6]), float(row[7])) * 2)
            velocity.append(float(row[8]))

    return centerline, roadside_inner, roadside_outer, width, velocity
            
    centerline.points.reverse()
    roadside_inner.points.reverse()
    roadside_outer.points.reverse()
    width.reverse()
    velocity.reverse()


def server():
    init()
    rospy.init_node('centerline_server')
    centerline_service = rospy.Service('get_centerline', Centerline, get_centerline)
    true_centerline_service = rospy.Service('get_true_centerline', Centerline, get_true_centerline)
    print("Ready to provide centerline.")
    rospy.spin()


if __name__ == "__main__":
    server()
