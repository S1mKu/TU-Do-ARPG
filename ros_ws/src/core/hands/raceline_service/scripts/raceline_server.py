#!/usr/bin/env python

from __future__ import print_function

from raceline_service.srv import Raceline, RacelineResponse
import rospy
import rospkg
import csv
import os

s_m = []
x_m = []
y_m = []
psi = []
kappa = []
v = []
a = []

def get_raceline(req):
    response = RacelineResponse()
    response.s_m = s_m
    response.x_m = x_m
    response.y_m = y_m
    response.psi = psi
    response.kappa = kappa
    response.v = v
    response.a = a

    return response


def loadPolygon():
    path = os.path.join(
        os.path.dirname(__file__),
        "..",
        "data",
        "traj_race_cl.csv"
    )

    with open(path, 'r') as csv_stream:
        reader = csv.reader(csv_stream)

        # skip first three lines
        next(reader)
        next(reader)
        next(reader)

        for row in reader:
            # s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2

            s_m.append(float(row[0]))
            x_m.append(float(row[1]))
            y_m.append(float(row[2]))
            psi.append(float(row[3]))
            kappa.append(float(row[4]))
            v.append(float(row[5]))
            a.append(float(row[6]))


def server():
    print("Preparing to provide raceline. 1")
    loadPolygon()
    print("Preparing to provide raceline. 2")
    rospy.init_node('raceline_service')
    print("Preparing to provide raceline. 3")
    s = rospy.Service('/get_raceline', Raceline, get_raceline)
    print("Ready to provide raceline.")
    rospy.spin()


if __name__ == "__main__":
    server()
