#!/usr/bin/env python3.8

from time_optimal_do_mpc_visualization import Visualization
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import PolygonStamped
from time_optimal_do_mpc.msg import drive_param
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from centerline_service.srv import Centerline
from tf import TransformListener
from eyes_msgs.msg import SegmentList
import rospy
import json
import numpy as np
import os
import do_mpc
import matplotlib.pyplot as plt

# mpc imports
from time_optimal_mpc.template_model import template_model
from time_optimal_mpc.template_mpc import template_mpc
from time_optimal_mpc.utils.interpolation import Interpolation
from time_optimal_mpc.utils.obstacle_parameterization import ObstacleParameterization
from geometry_msgs.msg import PointStamped
from shapely.geometry import Polygon

# global variables
odom = {}
curve = {}
interpolation: Interpolation
obstacles: ObstacleParameterization
tl: TransformListener
viz: Visualization
parameter = {}
last_s = 0   
current_s = 0           # for projection onto curve
last_odom = {}          # for visualization
stepSize = 0.01         # for projection onto curve # TODO what to choose?
vInit = 1               # inital velocity before mpc starts running

# debug flags
logging = False         # Flag for logging, use for debugging only slows mpc down
visualization = False   # Flag for visualization, use for debugging only slows mpc down
plot = False            # Flag for plotting, use for debugging only slows mpc down
timing = False          # Flag for timing of execution, use for debugging only slows mpc down


def get_centerline_response():
    rospy.wait_for_service('get_centerline')
    try:
        get_centerline = rospy.ServiceProxy('get_centerline', Centerline)
        resp1 = get_centerline()
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def odom_callback(data):
    global odom
    odom = data

def obstacle_callback(data):
    global obstacles

    poly = []
    bounding_boxes = []

    max_dist = parameter['mpc']['n_horizon'] * parameter['mpc']['t_step']
    for segment in data.segments:
        if segment is None or len(segment.points) < 3:
            continue # ignore obstacles with only 2 points
        
        close = False

        stampedPoints = []

        min_x = segment.points[0].x
        min_y = segment.points[0].y
        max_x = segment.points[0].x
        max_y = segment.points[0].y
        for pt in segment.points:

            if pt.x < min_x:
                min_x = pt.x
            elif pt.x > max_x:
                max_x = pt.x
            if pt.y < min_y:
                min_y = pt.y
            elif pt.y > max_y:
                max_y = pt.y

            if not close and np.sqrt(pt.x**2 + pt.y**2).real < max_dist:
                close = True

        # bounding box should be min 2times t_step in width
        if max_x - min_x < parameter['mpc']['t_step']*2:
            max_x = min_x + parameter['mpc']['t_step']*2

        if max_y - min_y < parameter['mpc']['t_step']*2:
            max_y = min_y + parameter['mpc']['t_step']*2

        header = Header()
        header.frame_id = "ego_racecar/base_link"

        stampedPoints.append(get_stamped_point(min_x, min_y, header))
        stampedPoints.append(get_stamped_point(min_x, max_y, header))
        stampedPoints.append(get_stamped_point(max_x, max_y, header))
        stampedPoints.append(get_stamped_point(max_x, min_y, header))

        # only consider obstacles that are in range of prediction horizon
        if close:
            # coordinate transformmation to map frame
            poly = [[tl.transformPoint("map", point).point.x, tl.transformPoint("map", point).point.y] for point in stampedPoints]
            shapely_poly = Polygon(poly)

            if visualization:
                viz.visualize_obstacle_poly(poly)

            bounding_boxes.append(shapely_poly)

    obstacles.polygons = bounding_boxes

def get_stamped_point(x, y, header):
    stamped = PointStamped()
    stamped.header = header
    stamped.point.x = x
    stamped.point.y = y
    return stamped

def get_state():
    # get current state of model for optimization
    # States are: X = [e_y, e_psi, v, t]
    global last_s
    global last_odom
    global currentS
    last_odom = odom # get current odom in case it changes in this function

    currentXY = [last_odom.pose.pose.position.x, last_odom.pose.pose.position.y]
    currentS = interpolation.projectOntoCurve(last_s, currentXY, stepSize)

    # set lastS for next projection
    global obstacles
    obstacles.current_s = currentS
    interpolation.last_s = currentS
    last_s = currentS

    # e_y = interpolation.calculateNormDistance(currentXY, currentS)
    e_y = interpolation.calculateNormDistance_alt(currentXY, currentS, stepSize)

    # get angle and transform to euler angles
    quaternion = (
        last_odom.pose.pose.orientation.x,
        last_odom.pose.pose.orientation.y,
        last_odom.pose.pose.orientation.z,
        last_odom.pose.pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    currentHeading = euler[2]
    headingS = interpolation.getHeadingOfCurveSingle(currentS)    

    anglediff = currentHeading - headingS
    # limit angle difference to [-pi, pi] 
    if anglediff > np.pi:
        anglediff -= 2* np.pi 
    if anglediff < -np.pi:
        anglediff += 2 * np.pi

    e_psi = anglediff

    velocity = last_odom.twist.twist.linear.x

    # We work with relative time so t0 is allways 0 in prediction 
    time = 0

    if logging:
        rospy.loginfo('CurrentS:                   %s', str(currentS))
        rospy.loginfo('CurrentS:                   %s', str(currentS))
        rospy.loginfo('CurrentHeading:             %s', str(currentHeading))
        rospy.loginfo('headingS:                   %s', str(headingS))
        rospy.loginfo('anglediff:                  %s', str(anglediff))
        rospy.loginfo('State:                      %s', str([e_y,e_psi,velocity,time]))

    return np.array([e_y,e_psi,velocity,time])

def start_node():
    #Init node
    rospy.init_node('time_optimal_do_mpc', anonymous=True)
    # Subscriber
    rospy.Subscriber('/odom', Odometry, odom_callback)

    # Publisher for control commands
    pub_ctrl = rospy.Publisher('/input/drive_param/autonomous', drive_param, queue_size=10) # TODO queue size? 

    # Init visualization
    global viz
    viz = Visualization()

    # Load MPC parameter from json file
    file_to_open = os.path.join(os.path.dirname(__file__), 'time_optimal_mpc', 'mpc_parameter.json')
    with open(file_to_open, 'r') as f:
        global parameter
        parameter = json.load(f)

    # get centerline
    centerline_response = get_centerline_response()
    centerline = centerline_response.centerline
    widths = centerline_response.width

    # Take minimum width from all as a lower bound, can be optimized for the future
    min_width = min(widths)*2

    # get interpolation curve of polygon from centerline
    # parse polygon for interpolation
    p = [[],[]]
    for point in centerline.polygon.points:
        p[0].append(point.x)
        p[1].append(point.y)

    # close polygon
    p[0].append(p[0][0])
    p[1].append(p[1][0])

    # reverse polynom
    p[0].reverse()
    p[1].reverse()

    # init interpolation
    global interpolation
    interpolation = Interpolation()
    interpolation.interpolatePolygon(p)

    # init obstacle avoidance
    global tl
    tl = TransformListener()
    global obstacles
    obstacles = ObstacleParameterization()
    obstacles.parameter = parameter
    obstacles.min_width = min_width
    obstacles.viz = viz
    rospy.Subscriber('/scan/final', SegmentList, obstacle_callback)

    # get model and mpc
    model = template_model(parameter)
    mpc = template_mpc(model, parameter, interpolation, min_width, obstacles)

    # get current state of car
    # TODO wait till odom is not undefined if this node starts before odom node
    currentXY = [odom.pose.pose.position.x, odom.pose.pose.position.y]
    global last_s
    last_s = interpolation.getInitialS(currentXY, stepSize)
    x0 = get_state()

    # visualization of curve
    if plot:
        viz.plot_interpolation(interpolation, currentXY)
    if visualization:
        viz.visualize_center_line(centerline_response.centerline)
        viz.visualize_interpolation(interpolation)
        viz.visualize_normal(interpolation)

    # accelerate before starting mpc because model is not defined for velocity = 0
    u0 = np.array([vInit + 1,0])
    while x0[2] < vInit:   
        ctrl_msg = drive_param()
        ctrl_msg.velocity = u0[0]
        ctrl_msg.angle = u0[1]
        pub_ctrl.publish(ctrl_msg)
        x0 = get_state()

    # Set initial state and input
    mpc.x0 = x0
    mpc.u0 = u0
    mpc.z0 = np.array([0,2])
    last_input = u0
    last_input_time = rospy.get_rostime()
    mpc.set_initial_guess()

    # main loop where the magic happens and the mpc runs ;)
    while not rospy.is_shutdown():
        t1 = rospy.get_rostime()

        # get current state from messages and interpolate it
        x0 = get_state()
        """
        TODO
        Somehow x0 stays allways the same in optimized output mpc.opt_x_num, 
        I don't know if this is only a bug in opt_x_num of if x0 really does not get set while solving.
        if this here get commented in howewer x0 is also set in opt_x_num.
        """
        mpc.x0 = x0
        mpc.u0 = u0
        mpc.set_initial_guess()

        t2 = rospy.get_rostime()

        # call MPC with current state
        u0 = mpc.make_step(x0)

        t3 = rospy.get_rostime()

        # TODO limiting the inputs (especially stering angle) by time again could maybe improve swinging of car
        # limit control intputs by time
        now = rospy.get_rostime()

        a_max = parameter['model']['a_max']
        sv_max = parameter['model']['sv_max']

        # # calculate max input change
        delta_t = now - last_input_time
        delta_t_s = delta_t.to_sec()
        sv_max_delta_t = (sv_max*delta_t_s)

        # limit steering input
        diff_u_delta = u0[1] - last_input[1]

        if diff_u_delta >= 0:
            sv_max_delta_t = (sv_max*delta_t_s)
        else:
            sv_max_delta_t = -(sv_max*delta_t_s)

        if np.abs(diff_u_delta) > np.abs(sv_max_delta_t):
            u0[1] = last_input[1] + sv_max_delta_t

        last_input = u0
        last_input_time = now

        # publish new control inputs
        ctrl_msg = drive_param()
        ctrl_msg.velocity = u0[0]
        ctrl_msg.angle = u0[1]
        pub_ctrl.publish(ctrl_msg)

        t4 = rospy.get_rostime()

        if logging:
            rospy.loginfo('Optimized control input:    %s', str([u0[0][0], u0[1][0]]))

        # visualize prediction
        if visualization:
            viz.visualize_prediction(mpc, interpolation, last_s, parameter, x0, last_odom)

        # print timing
        if timing:
            t5 = rospy.get_rostime()
            rospy.logwarn('[MPC]===Timing================')
            rospy.logwarn('[MPC] getState time:    %s[ms]', (t2-t1).to_sec()*1000)
            rospy.logwarn('[MPC] solver time:      %s[ms]', (t3-t2).to_sec()*1000)
            rospy.logwarn('[MPC] publish time:     %s[ms]', (t4-t3).to_sec()*1000)
            rospy.logwarn('[MPC] viz time:         %s[ms]', (t5-t4).to_sec()*1000)
            rospy.logwarn('[MPC] total loop time:  %s[ms]', (t5-t1).to_sec()*1000)

if __name__ == '__main__':

    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
