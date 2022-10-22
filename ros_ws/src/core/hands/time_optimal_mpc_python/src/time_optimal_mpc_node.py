#!/usr/bin/env python3.8

from time_optimal_mpc_visualization import Visualization
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import PolygonStamped, PoseWithCovarianceStamped
from time_optimal_mpc_python.msg import drive_param
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from centerline_service.srv import Centerline
from tf import TransformListener
from eyes_msgs.msg import SegmentList, Segment, ObstacleList, Obstacle
import rospy
import json
import numpy as np
import os
import matplotlib.pyplot as plt
import decimal
import math
import forcespro
import forcespro.nlp

# mpc imports
from time_optimal_mpc.utils import config
from time_optimal_mpc.utils.interpolation import Interpolation
from time_optimal_mpc.utils.obstacle_parameterization import ObstacleParameterization
from geometry_msgs.msg import PointStamped
from shapely.geometry import Polygon

# global variables
odom = {}
pose = {}
curve = {}
interpolation: Interpolation
obstacles: ObstacleParameterization = ObstacleParameterization()
tl: TransformListener
viz: Visualization
parameter = {}
last_s = 0   
current_s = 0           # for projection onto curve
last_odom = {}          # for visualization
last_pose = {}
stepSize = 0.01         # for projection onto curve
vInit = 1               # inital velocity before mpc starts running

# debug flags
logging = False         # Flag for logging, use for debugging only slows mpc down
visualization = False   # Flag for visualization, use for debugging only slows mpc down
plot = False            # Flag for plotting, use for debugging only slows mpc down
timing = True          # Flag for timing of execution, use for debugging only slows mpc down


def subdivide_segment(segment):
    segment_list = []

    p1 = segment.points[0]
    p2 = segment.points[-1]

    l = math.sqrt((p1.x - p2.x)**2.0 + (p1.y - p2.y)**2.0)
    n = len(segment.points)

    step = math.floor(n / l)
    seg = None

    print(f"subdivide_segment: {n} {l} {step}")

    for i in range(n):
        if i % step == 0:
            seg = Segment()
            segment_list.append(seg)
        
        seg.points.append(segment.points[i])

    print(f"subdivide_segment list len: {len(segment_list)}")

    return segment_list


def get_bounding_box(points):
    poly = []

    p_start = points[0]
    p_end = points[-1]

    dx = p_end.x - p_start.x
    dy = p_end.y - p_start.y
    line_norm = math.sqrt(dx**2.0 + dy**2.0)
    dx /= line_norm
    dy /= line_norm

    nx = dy
    ny = -dx
    norm = math.sqrt(nx**2.0 + ny**2.0)
    nx /= norm
    ny /= norm

    b_min = 0
    b_max = 0

    for i in range(1, len(points)):
        px = points[i].x
        py = points[i].y

        b = (((px - p_start.x) * dy) - ((py - p_start.y) * dx)) / ((nx * dy) - (ny * dx))

        if b > b_max:
            b_max = b
        elif b < b_min:
            b_min = b

    e1x = p_start.x + nx * (b_min - 0.2) + (-dx * 0.1)
    e1y = p_start.y + ny * (b_min - 0.2) + (-dy * 0.1)

    e2x = p_start.x + nx * (b_max + 0.2) + (-dx * 0.1)
    e2y = p_start.y + ny * (b_max + 0.2) + (-dy * 0.1)

    e3x = p_end.x + nx * (b_max + 0.2) + (dx * 0.1)
    e3y = p_end.y + ny * (b_max + 0.2) + (dy * 0.1)

    e4x = p_end.x + nx * (b_min - 0.2) + (dx * 0.1)
    e4y = p_end.y + ny * (b_min - 0.2) + (dy * 0.1)

    poly.append([e1x, e1y])
    poly.append([e2x, e2y])
    poly.append([e3x, e3y])
    poly.append([e4x, e4y])
    poly.append([e1x, e1y])

    return poly

def get_centerline_response():
    rospy.wait_for_service('/get_centerline')
    try:
        get_centerline = rospy.ServiceProxy('/get_centerline', Centerline)
        resp1 = get_centerline()
        return resp1
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s" % e)

def pose_callback(data):
    global pose
    pose = data

def odom_callback(data):
    """
    only for retrieving velocity information
    see pose_callback(...) for retrieval of position information
    """
    global odom
    odom = data

def obstacle_callback(data):
    global obstacles

    poly = []
    bb_polygons = []
    bounding_boxes = []

    for obstacle in data.obstacles:
        segment = obstacle.segment

        sub_segments = subdivide_segment(segment)

        for sub_segment in sub_segments:
            #TODO: only consider obstacles that are in range of prediction horizon
            poly = get_bounding_box(sub_segment.points)
            shapely_poly = Polygon(poly)
            bounding_boxes.append(shapely_poly)
            bb_polygons.append(poly)

    obstacles.polygons = bounding_boxes

    if visualization:
        viz.visualize_obstacles(data)
        viz.visualize_obstacle_bbs(bb_polygons)

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
    global last_pose
    global currentS
    last_odom = odom # get current odom in case it changes in this function

    last_pose = pose

    currentXY = [last_pose.pose.pose.position.x, last_pose.pose.pose.position.y]
    currentS = interpolation.projectOntoCurve(last_s, currentXY, stepSize)

    # set lastS for next projection
    # global obstacles
    # obstacles.current_s = currentS
    interpolation.last_s = currentS
    last_s = currentS

    # e_y = interpolation.calculateNormDistance(currentXY, currentS)
    e_y = interpolation.calculateNormDistance_alt(currentXY, currentS, stepSize)

    # get angle and transform to euler angles
    quaternion = (
        last_pose.pose.pose.orientation.x,
        last_pose.pose.pose.orientation.y,
        last_pose.pose.pose.orientation.z,
        last_pose.pose.pose.orientation.w)
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

    # We work with relative time so t0 is always 0 in prediction 
    time = 0

    if logging:
        rospy.loginfo('CurrentS:                   %s', str(currentS))
        rospy.loginfo('CurrentHeading:             %s', str(currentHeading))
        rospy.loginfo('headingS:                   %s', str(headingS))
        rospy.loginfo('anglediff:                  %s', str(anglediff))
        rospy.loginfo('State:                      %s', str([e_y,e_psi,velocity,time]))
        rospy.loginfo('\n')

    return np.array([e_y,e_psi,velocity,last_delta,time])

def start_node():
    #Init node
    rospy.init_node('time_optimal_mpc_python', anonymous=True)

    # Odom topic
    odom_topic = rospy.get_namespace() + "odom"

    # Subscriber
    rospy.Subscriber(odom_topic, Odometry, odom_callback)
    rospy.Subscriber("pose", PoseWithCovarianceStamped, pose_callback)

    # Publisher for control commands
    pub_ctrl = rospy.Publisher('input/drive_param/autonomous', drive_param, queue_size=10)

    # Init visualization
    global viz
    viz = Visualization()

    # get centerline
    centerline_response = get_centerline_response()
    centerline = centerline_response.centerline
    widths = centerline_response.width

    # Take minimum width from all as a lower bound, can be optimized for the future
    min_width = min(widths)

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
    obstacles.config = config
    obstacles.min_width = min_width
    obstacles.viz = viz
    rospy.Subscriber('obstacle_tracking/obstacles', ObstacleList, obstacle_callback)

    # get model and mpc
    solver = forcespro.nlp.Solver.from_directory(os.path.join(os.path.dirname(__file__), "time_optimal_mpc/utils/FORCESNLPsolver/"))

    # wait till odom is not undefined anymore
    while odom == {} or pose == {}:
        rospy.sleep(0.1)
        rospy.loginfo("wait for odom and pose")

    # get current state of car
    currentXY = [pose.pose.pose.position.x, pose.pose.pose.position.y]
    global last_s
    last_s = interpolation.getInitialS(currentXY, stepSize)
    x0 = get_state(0.)

    # visualization of curve
    if plot:
        viz.plot_interpolation(interpolation, currentXY)
    if visualization:
        viz.visualize_center_line(centerline_response.centerline)
        viz.visualize_interpolation(interpolation)
        viz.visualize_normal(interpolation)

    # accelerate before starting mpc because model is not defined for velocity = 0
    u0 = np.array([vInit + 1, 0])
    while x0[2] < vInit:   
        ctrl_msg = drive_param()
        ctrl_msg.velocity = u0[0]
        ctrl_msg.angle = u0[1]
        pub_ctrl.publish(ctrl_msg)
        x0 = get_state(ctrl_msg.angle)
        rospy.sleep(0.1)

    N = config.N # Horizon length
    nvar = len(config.I['inputs']) + len(config.I['states']) # number of variables
    nu = len(config.I['inputs']) # number of inputs to shift indices of states

    # Set initial guess
    x0i_mpc = np.array([0.,0.,0.,0.,0.,0.,0.,0.]).reshape(nvar,1)
    x0_mpc = np.transpose(np.tile(x0i_mpc, (1, N)))
    last_delta = 0.
    last_input = rospy.get_rostime()

    # Debug counter, can be removed for "production"
    loop_cnt = 0    # total loop count
    iter_cnt = 0    # max iterations reached count
    err_cnt = 0     # bad exitflag count

    # main loop where the magic happens and the mpc runs ;)
    while not rospy.is_shutdown():
        t1 = rospy.get_rostime()

        loop_cnt +=1

        # get current state from messages and interpolate it and set it for solver
        x0 = get_state(last_delta)
        xinit_mpc = np.transpose(x0)

        # set runtime parameter
        all_parameters = np.array([])
        for i in range(config.N):
            s = last_s + i * config.integrator_ts

            # calculate obstacles bounds for each s here!
            # ob_width = (ob_ub - ob_lb)/2  => width of obstacle
            # e_ob = ob_lb + ob_width/2     => error of obstacle to center of track
            widths, e_obs_list = obstacles.getParameterisation(interpolation, s)

            if len(widths) > 0:
                e_ob_s = e_obs_list[0]
                ob_width_s = widths[0]

                if visualization:
                    viz.visualize_obstacle_param(interpolation, s, e_ob_s, ob_width_s)
                    
            else:
                # no obstacle in track so place bounds outside of track
                e_ob_s = 2 * min_width
                ob_width_s = 0


            rho_s = 1 / interpolation.getLocalCurvatureOfCurveSingle(s)
            params = np.array([rho_s, min_width, e_ob_s, ob_width_s])
            all_parameters = np.append(all_parameters, params)

        all_parameters = np.transpose(all_parameters)

        problem = {"x0": x0_mpc,
                "xinit": xinit_mpc,
                "all_parameters": all_parameters}

        t2 = rospy.get_rostime()

        # Time to solve the NLP!
        output, exitflag, info = solver.solve(problem)

        t3 = rospy.get_rostime()

        # only execute controls if solver did find solution
        # if exitflag >= 0 :
        # calculate desired input velocity from reverting the p controller of the sim with the optimized acceleration (see dynamic_model.py of sim for reference)

        a_des = output['x01'][config.I['accel']]    # desired acceleration
        v_cur = x0[config.I['velocity'] - nu]               # current velocity

        # calculate kp
        if a_des > 0:
            kp = 2 * config.a_ub_sim/config.v_ub_sim
        else:
            kp = 2 * config.a_ub_sim/(-config.v_lb_sim)

        v_des = v_cur + a_des/kp    # optimal desired speed
        
        # calculate desired delta from last delta , optimized steering speed and time passed
        delta_des = last_delta + output['x01'][config.I['steer_rate']] * (rospy.get_rostime()-last_input).to_sec()

        # publish new control inputs
        ctrl_msg = drive_param()
        ctrl_msg.velocity = v_des
        ctrl_msg.angle = delta_des
        last_delta = ctrl_msg.angle
        last_input = rospy.get_rostime()
        pub_ctrl.publish(ctrl_msg)

        t4 = rospy.get_rostime()

        if exitflag < 0:
            err_cnt +=1
            rospy.logwarn('Bad exitflag: %s', str(exitflag))
            rospy.logwarn('Error count: %s of %s', err_cnt, loop_cnt)
        if exitflag == 0:
            iter_cnt +=1
            rospy.logwarn('Maximum iterations reached. Exitflag: %s', str(exitflag))
            rospy.logwarn('Max iter count: %s of %s', iter_cnt, loop_cnt)
            # TODO analyze iteration count 

        if logging:
            rospy.loginfo('Optimized control rates:    %s', str([output['x01'][config.I['accel']], output['x01'][config.I['steer_rate']]]))
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
            viz.visualize_prediction(output, interpolation, last_s)
            viz.visualize_state(interpolation, last_s, x0)

        # print timing
        if timing:
            t5 = rospy.get_rostime()
            rospy.logwarn('[MPC]===Timing==================')
            rospy.logwarn('[MPC] getState time:      %s[ms]', (t2-t1).to_sec()*1000)
            rospy.logwarn('[MPC] solver time (mesu): %s[ms]', (t3-t2).to_sec()*1000)
            rospy.logwarn('[MPC] solver time (info): %s', info.solvetime*1000)
            rospy.logwarn('[MPC] fe vals time:       %s', info.fevalstime*1000)
            rospy.logwarn('[MPC] publish time:       %s[ms]', (t4-t3).to_sec()*1000)
            rospy.logwarn('[MPC] viz time:           %s[ms]', (t5-t4).to_sec()*1000)
            rospy.logwarn('[MPC] total loop time:    %s[ms]', (t5-t1).to_sec()*1000)

if __name__ == '__main__':

    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
