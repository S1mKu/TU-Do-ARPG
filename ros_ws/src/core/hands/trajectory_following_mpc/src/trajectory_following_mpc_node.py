#!/usr/bin/env python3.8

from visualization import Visualization
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import PolygonStamped
from time_optimal_mpc_python.msg import drive_param
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from centerline_service.srv import Centerline
from optimal_trajectory_service.srv import Trajectory
from tf import TransformListener
from eyes_msgs.msg import SegmentList
import rospy
import json
import numpy as np
import os
import matplotlib.pyplot as plt
import decimal

import forcespro
import forcespro.nlp

# mpc imports
from utils import config
from utils.interpolation import Interpolation
from geometry_msgs.msg import PointStamped
from shapely.geometry import Polygon, Point

# global variables
odom = {}
curve = {}
interpolationTrajectory: Interpolation
interpolationCenterline: Interpolation
#obstacles: ObstacleParameterization
tl: TransformListener
viz: Visualization
parameter = {}
last_s = 0   
current_s = 0           # for projection onto curve
last_odom = {}          # for visualization
stepSize = 0.01         # for projection onto curve # TODO what to choose?
vInit = 1               # inital velocity before mpc starts running

# debug flags
logging = True         # Flag for logging, use for debugging only slows mpc down
visualization = True   # Flag for visualization, use for debugging only slows mpc down
plot = False            # Flag for plotting, use for debugging only slows mpc down
timing = False          # Flag for timing of execution, use for debugging only slows mpc down


def get_centerline_response():
    rospy.wait_for_service('/get_centerline')
    try:
        get_centerline = rospy.ServiceProxy('/get_centerline', Centerline)
        resp1 = get_centerline()
        return resp1
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s" % e)

def get_trajectory_response():
    rospy.wait_for_service('/get_trajectory')
    try:
        get_trajectory = rospy.ServiceProxy('/get_trajectory', Trajectory)
        resp1 = get_trajectory()
        return resp1
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s" % e)

def odom_callback(data):
    global odom
    odom = data

def get_stamped_point(x, y, header):
    stamped = PointStamped()
    stamped.header = header
    stamped.point.x = x
    stamped.point.y = y
    return stamped

def get_state(last_delta):
    # get current state of model for optimization
    # States are: X = [e_y, e_psi, v, t]
    global last_s
    global last_odom
    global currentS
    last_odom = odom # get current odom in case it changes in this function

    currentXY = [last_odom.pose.pose.position.x, last_odom.pose.pose.position.y]
    currentS = interpolationTrajectory.projectOntoCurve(last_s, currentXY, stepSize)

    # set lastS for next projection
    # global obstacles
    # obstacles.current_s = currentS
    interpolationTrajectory.last_s = currentS
    last_s = currentS

    # e_y = interpolation.calculateNormDistance(currentXY, currentS)
    e_y = interpolationTrajectory.calculateNormDistance_alt(currentXY, currentS, stepSize)

    # get angle and transform to euler angles
    quaternion = (
        last_odom.pose.pose.orientation.x,
        last_odom.pose.pose.orientation.y,
        last_odom.pose.pose.orientation.z,
        last_odom.pose.pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    currentHeading = euler[2]
    headingS = interpolationTrajectory.getHeadingOfCurveSingle(currentS)    

    anglediff = currentHeading - headingS
    # limit angle difference to [-pi, pi] 
    if anglediff > np.pi:
        anglediff -= 2* np.pi 
    if anglediff < -np.pi:
        anglediff += 2 * np.pi

    e_psi = anglediff

    velocity = last_odom.twist.twist.linear.x

    # We work with relative time so t0 is always 0 in prediction 
    time = 0

    # if logging:
    #     rospy.loginfo('CurrentS:                   %s', str(currentS))
    #     rospy.loginfo('CurrentHeading:             %s', str(currentHeading))
    #     rospy.loginfo('headingS:                   %s', str(headingS))
    #     rospy.loginfo('anglediff:                  %s', str(anglediff))
    #     rospy.loginfo('State:                      %s', str([e_y,e_psi,velocity,time]))
    #     rospy.loginfo('\n')

    return np.array([e_y,e_psi,velocity,last_delta,time])

def get_border_distances(s, poly_inner, poly_outer):
    global interpolationCenterline
    
    XY = interpolationTrajectory.evaluateInterpolation(s)
    shapely_point = Point(XY[0], XY[1])

    temp_inner = [[stamped_point.x, stamped_point.y] for stamped_point in poly_inner.points]
    temp_outer = [[stamped_point.x, stamped_point.y] for stamped_point in poly_outer.points]
    
    shapely_inner = Polygon(temp_inner)
    shapely_outer = Polygon(temp_outer)

    dist_i = shapely_point.distance(shapely_inner)
    dist_o = shapely_outer.exterior.distance(shapely_point)

    return dist_i, dist_o


def start_node():
    #Init node
    rospy.init_node('trajectory_following_mpc_python', anonymous=True)

    # Odom topic
    odom_topic = rospy.get_namespace() + "odom"#

    # Subscriber
    rospy.Subscriber(odom_topic, Odometry, odom_callback)

    # Publisher for control commands
    pub_ctrl = rospy.Publisher('input/drive_param/autonomous', drive_param, queue_size=10) # TODO queue size? 

    # Init visualization
    global viz
    viz = Visualization()

    # get centerline
    centerline_response = get_centerline_response()
    centerline = centerline_response.centerline
    widths = centerline_response.width

    # get trajectory
    trajectory_response = get_trajectory_response()
    trajectory = trajectory_response.raceline

    # Take minimum width from all as a lower bound, can be optimized for the future
    min_width = min(widths)
    # min_width = 0.5

    # get interpolation curve of polygon from centerline
    # parse polygon for interpolation
    p = [[],[]]
    for point in centerline.polygon.points:
        p[0].append(point.x)
        p[1].append(point.y)

    # get interpolation curve of polygon from centerline
    # parse polygon for interpolation
    pT = [[],[]]
    for point in trajectory.polygon.points:
        pT[0].append(point.x)
        pT[1].append(point.y)

    # close polygon
    p[0].append(p[0][0])
    p[1].append(p[1][0])

    # reverse polynom
    p[0].reverse()
    p[1].reverse()

    # close polygon
    pT[0].append(p[0][0])
    pT[1].append(p[1][0])

    # reverse polygon
    # pT[0].reverse()
    # pT[1].reverse()

    # init interpolation
    global interpolationCenterline
    interpolationCenterline = Interpolation()
    interpolationCenterline.interpolatePolygon(p)

    # init interpolation
    global interpolationTrajectory
    interpolationTrajectory = Interpolation()
    interpolationTrajectory.interpolatePolygon(pT)
    v_list = trajectory_response.velocity
    s_list = trajectory_response.s

    interpolationTrajectory.interpolateVelocities(s_list, v_list)

    rospy.loginfo("length of trajectory curve: " + str(interpolationTrajectory.lengthOfCurve))

    poly_outer, poly_inner = interpolationCenterline.getBorderPolygons(1000, min_width)

    # init obstacle avoidance
    global tl
    tl = TransformListener()

    # get model and mpc
    solver = forcespro.nlp.Solver.from_directory(os.path.join(os.path.dirname(__file__), "utils/FORCESNLPsolver/"))

    # wait till odom is not undefined anymore
    while odom == {}:
        rospy.sleep(0.1)
        rospy.loginfo("wait for odom")

    # get current state of car
    currentXY = [odom.pose.pose.position.x, odom.pose.pose.position.y]
    global last_s
    last_s = interpolationTrajectory.getInitialS(currentXY, stepSize)
    x0 = get_state(0.)

    # visualization of curve
    if plot:
        viz.plot_interpolation(interpolationTrajectory, currentXY)
    if visualization:
        viz.visualize_center_line(centerline_response.centerline)
        viz.visualize_inner_line(poly_inner)
        viz.visualize_outer_line(poly_outer)
        viz.visualize_race_line(trajectory_response.raceline)
        viz.visualize_interpolation(interpolationTrajectory)
        viz.visualize_normal(interpolationTrajectory)

    # accelerate before starting mpc because model is not defined for velocity = 0
    u0 = np.array([vInit + 1,0])
    while x0[2] < vInit:   
        ctrl_msg = drive_param()
        ctrl_msg.velocity = u0[0]
        ctrl_msg.angle = u0[1]
        pub_ctrl.publish(ctrl_msg)
        x0 = get_state(ctrl_msg.angle)
        rospy.sleep(0.1)

    # Set initial state and input
    # mpc.x0 = x0
    # mpc.u0 = u0
    # mpc.z0 = np.array([0,2])
    # last_input = u0
    # last_input_time = rospy.get_rostime()
    # mpc.set_initial_guess()

    N = config.N # Horizon length
    nvar = 7 # number of variables
    npar = 4 # number of parameter
    last_delta = 0.

    # main loop where the magic happens and the mpc runs ;)
    while not rospy.is_shutdown():
        t1 = rospy.get_rostime()

        # get current state from messages and interpolate it
        x0 = get_state(last_delta)

        # Set initial guess to start solver from
        x0i_mpc = np.array([0.,0.,0.,0.,0.,0.,0.]).reshape(nvar,1)
        x0_mpc = np.transpose(np.tile(x0i_mpc, (1, N)))
        xinit_mpc = np.transpose(x0)

        all_parameters = np.array([])
        for i in range(config.N):
            s = last_s + i * config.integrator_ts

            # # calculate obstacles bounds for each s here!
            # # ob_width = (ob_ub - ob_lb)/2  => width of obstacle
            # # e_ob = ob_lb + ob_width/2     => error of obstacle to center of track
            # # try: 
            # widths, e_obs_list = obstacles.getParameterisation(interpolationTrajectory, s)

            # if len(widths) == 0:
            #     # no obstacle in track so place bounds outside of track
            #     e_ob_s = 2 * min_width
            #     ob_width_s = 0
            # else:
            #     e_ob_s = e_obs_list[0]
            #     ob_width_s = widths[0]
            # except Exception as e: # work on python 3.x
            #     print('Exception in Obstacles: ' + str(e))
            #     # place bounds outside of track
            #     e_ob_s = 2 * min_width
            #     ob_width_s = 0

            dist_i, dist_o = get_border_distances(s, poly_inner, poly_outer)
            v_ref = interpolationTrajectory.evaluateVelocity(s)

            rho_s = 1 / interpolationTrajectory.getLocalCurvatureOfCurveSingle(s)
            params = np.array([rho_s, dist_i, dist_o, v_ref])
            all_parameters = np.append(all_parameters, params)

        all_parameters = np.transpose(all_parameters)


        problem = {"x0": x0_mpc,
                "xinit": xinit_mpc,
                "all_parameters": all_parameters}

        t2 = rospy.get_rostime()

        # Time to solve the NLP!
        output, exitflag, info = solver.solve(problem)

        t3 = rospy.get_rostime()

        # calculate desired input velocity from reverting the p controller of the sim with the optimized acceleration (see dynamic_model.py of sim for reference)
        # TODO maybe if steering is also off, take steering speed and calculate new angle from old angle, speed and time from last input or something like this? But for now seams to work fine like this

        a_des = output['x01'][0]    # desired acceleration
        v_cur = x0[2]               # current velocity

        # calculate kp
        if a_des > 0:
            kp = 2 * config.a_ub_sim/config.v_ub_sim
        else:
            kp = 2 * config.a_ub_sim/(-config.v_lb_sim)

        v_des = v_cur + a_des/kp    # optimal desired speed

        # publish new control inputs
        ctrl_msg = drive_param()
        ctrl_msg.velocity = v_des
        ctrl_msg.angle = output['x02'][5]
        last_delta = ctrl_msg.angle
        pub_ctrl.publish(ctrl_msg)


        t4 = rospy.get_rostime()

        if logging:
            rospy.loginfo('Optimized control input:    %s', str([ctrl_msg.velocity, ctrl_msg.angle]))
            rospy.loginfo('Exitflag:                   %s', str(exitflag))

            rospy.loginfo('Info:')
            rospy.loginfo('     it:                    %s', str(info.it))
            rospy.loginfo('     res_eq:                %s', str(info.res_eq))
            rospy.loginfo('     res_ineq:              %s', str(info.res_ineq))
            rospy.loginfo('     rsnorm:                %s', str(info.rsnorm))
            rospy.loginfo('     rcompnorm:             %s', str(info.rcompnorm))
            rospy.loginfo('     pobj:                  %s', str(info.pobj))
            rospy.loginfo('     mu:                    %s', str(info.mu))
            rospy.loginfo('     solvetime:             %s', str(info.solvetime))
            rospy.loginfo('     fevalstime:            %s', str(info.fevalstime))
            rospy.loginfo('\n')

            # rospy.loginfo('Output:')
            # rospy.loginfo("n\t\ta\t\t\tdelta\t\t\te_y\t\t\te_psi\t\t\tv\t\t\tt")
            # rospy.loginfo("-"*144)

            # # To print without scientific notation
            # ctx = decimal.Context()
            # ctx.prec = 20

            # for x in output:
            #     line = "" + x + "\t"
            #     for num in output.get(x).tolist():
            #         y = ctx.create_decimal(repr(num))
            #         y = format(y, 'f')
            #         if(len(y)<16):
            #             y = y + ("0"*(16-len(y)))
            #         if(y[0]!='-'):
            #             y = "+" + y
            #         y = y[0:16]
            #         line = line + y + "\t"
            #     rospy.loginfo(line)
            # rospy.loginfo('\n')



        # visualize prediction
        if visualization:
            viz.visualize_prediction(output, interpolationTrajectory, last_s, parameter, x0, last_odom)
            viz.visualize_state(interpolationTrajectory, last_s, x0)

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
