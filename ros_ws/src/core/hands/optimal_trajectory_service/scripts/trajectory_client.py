#!/usr/bin/env python

from __future__ import print_function

import rospy
from optimal_trajectory_service.srv import Trajectory


def get_trajectory():
    rospy.wait_for_service('get_trajectory')
    try:
        get_centerline = rospy.ServiceProxy('get_trajectory', Trajectory)
        resp1 = get_trajectory()
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    print("Requesting trajectory")
    print(get_trajectory())
