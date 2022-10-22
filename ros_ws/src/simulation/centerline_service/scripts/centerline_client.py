#!/usr/bin/env python

from __future__ import print_function

import rospy
from centerline_service.srv import Centerline


def get_centerline():
    rospy.wait_for_service('get_centerline')
    try:
        get_centerline = rospy.ServiceProxy('get_centerline', Centerline)
        resp1 = get_centerline()
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    print("Requesting centerline")
    print(get_centerline())
