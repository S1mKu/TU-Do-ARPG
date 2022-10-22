
# from time_optimal_mpc.utils.obstacle_parameterization import ObstacleParameterization
from utils.interpolation import Interpolation
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA, Float32, Header
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import numpy as np
import rospy
import matplotlib.pyplot as plt
from utils import config

class Visualization:

    pub_centerl = {}
    pub_racel = {}
    pub_outerl = {}
    pub_innterl = {}
    pub_interp = {}
    pub_pred = {}
    pub_state = {}
    pub_odom = {}
    pub_obs = {}
    pub_v = {}
    pub_delta = {}
    pub_a = {}
    id = 0

    def __init__(self):
        self.pub_centerl = rospy.Publisher('/visualization/center_line', PolygonStamped, queue_size=10)
        self.pub_racel = rospy.Publisher('/visualization/race_line', PolygonStamped, queue_size=10)
        self.pub_outerl = rospy.Publisher('/visualization/outer_line', PolygonStamped, queue_size=10)
        self.pub_innerl = rospy.Publisher('/visualization/inner_line', PolygonStamped, queue_size=10)
        self.pub_interp = rospy.Publisher('/visualization/interpolation', PolygonStamped, queue_size=10)
        self.pub_interp_v = rospy.Publisher('/visualization/interpolation_v', Marker, queue_size=10)
        self.pub_pred = rospy.Publisher('/visualization/prediction', Path, queue_size=10)
        self.pub_state = rospy.Publisher('/visualization/pose_state', PoseStamped, queue_size=10)
        self.pub_odom = rospy.Publisher('/visualization/pose_odom', PoseStamped, queue_size=10)
        self.pub_norm = rospy.Publisher('/visualization/normal', PolygonStamped, queue_size=10)
        self.pub_obs = rospy.Publisher('/visualization/obs', Marker, queue_size=100)

        self.pub_v = rospy.Publisher('mpc/rviz/velocity', Float32, queue_size=10)
        self.pub_delta = rospy.Publisher('mpc/rviz/delta', Float32, queue_size=10)
        self.pub_a = rospy.Publisher('mpc/rviz/accel', Float32, queue_size=10)

    def visualize_center_line(self, center_line: PolygonStamped):
        msg = center_line
        msg.header.seq = 1
        msg.header.frame_id = 'map'
        self.pub_centerl.publish(msg)
        print('Centerline Published!')

    def visualize_outer_line(self, outer_line: Polygon):
        msg = PolygonStamped()
        msg.polygon = outer_line
        msg.header.seq = 1
        msg.header.frame_id = 'map'
        self.pub_outerl.publish(msg)
        print('Outerline Published!')

    def visualize_inner_line(self, inner_line: Polygon):
        msg = PolygonStamped()
        msg.polygon = inner_line
        msg.header.seq = 1
        msg.header.frame_id = 'map'
        self.pub_innerl.publish(msg)
        print('Innerline Published!')

    def visualize_race_line(self, race_line: PolygonStamped):
        msg = race_line
        msg.header.seq = 1
        msg.header.frame_id = 'map'
        self.pub_racel.publish(msg)
        print('Raceline Published!')

    def visualize_interpolation(self, interpolation: Interpolation):
        id = self.id

        msg = PolygonStamped()
        msg.header.seq = 1
        msg.header.frame_id = 'map'
        msg.header.stamp = rospy.Time.now()

        header = Header()
        header.seq = 1
        header.frame_id = 'map'
        header.stamp = rospy.Time.now()

        velocities = Marker()
        velocities.header = header

        velocities.type = Marker.LINE_LIST
        velocities.action = Marker.ADD


        velocities.id = ++id
        velocities.scale.x = 0.03
        velocities.scale.y = 0.03
        velocities.scale.z = 0.03
        velocities.color.r = 0.0
        velocities.color.g = 1.0
        velocities.color.b = 0.0
        velocities.color.a = 1.0
        velocities.ns = "velocities"

        n = 1000 # number of evaluation points
        for i in range(n):

            s = i*interpolation.lengthOfCurve/n

            XY = interpolation.evaluateInterpolation(s)

            point = Point32()
            point.x = XY[0]
            point.y = XY[1]
            point.z = 0

            msg.polygon.points.append(point)

            # Create Points for visualize predicted velocities (z axis vor speed)
            point1 = Point()
            point2 = Point()
            color = ColorRGBA()

            point1.x = XY[0]
            point1.y = XY[1]
            point1.z = 0

            v = interpolation.evaluateVelocity(s)

            point2.x = XY[0]
            point2.y = XY[1]
            point2.z = v / config.v_ub_sim

            # color.r = - (a - config.a_lb) / (config.a_ub - config.a_lb)
            # color.g = (a - config.a_lb) / (config.a_ub - config.a_lb)
            # color.b = 0
            # color.a = 1.0

            velocities.points.append(point1)
            velocities.points.append(point2)

        self.pub_interp.publish(msg)
        self.pub_interp_v.publish(velocities)
        print('Interpolation Published!')

    def visualize_prediction(self, solverOutput, interpolation:Interpolation, currentS, parameter, x0: np.ndarray, odom: Odometry):

        x0_pose = PoseStamped()

        xy_s = interpolation.evaluateInterpolation(currentS)
        psi_s = interpolation.getHeadingOfCurveSingle(currentS)

        e_y = x0[0]
        e_psi = x0[1]
        psi = e_psi + psi_s

        quaternion = quaternion_from_euler(0, 0, psi)

        x0_pose.pose.position.x = xy_s[0] - e_y * np.sin(psi_s)
        x0_pose.pose.position.y = xy_s[1] + e_y * np.cos(psi_s)
        x0_pose.pose.position.z = 0
        x0_pose.pose.orientation.x = quaternion[0]
        x0_pose.pose.orientation.y = quaternion[1]
        x0_pose.pose.orientation.z = quaternion[2]
        x0_pose.pose.orientation.w = quaternion[3]

        x0_pose.header.seq = 1
        x0_pose.header.frame_id = 'map'
        x0_pose.header.stamp = rospy.Time.now()

        self.pub_state.publish(x0_pose)

        n_horizon = config.N
        s_step = config.integrator_ts
        
        msg = Path()
        msg.header.seq = 1
        msg.header.frame_id = 'map'
        msg.header.stamp = rospy.Time.now()
        msg.poses.append(x0_pose)
        
        outputKeys = list(solverOutput.keys()) 

        for i in range(0, n_horizon):
            pose = PoseStamped()
            si = (currentS + i*s_step)

            xy_si = interpolation.evaluateInterpolation(si)
            psi_si = interpolation.getHeadingOfCurveSingle(si)

            # e_y_i = mpc.opt_x_num['_x', i , 0, 0, 'e_y']
            e_y_i = solverOutput[outputKeys[i]][2]

            pose.pose.position.x = xy_si[0] - e_y_i * np.sin(psi_si)
            pose.pose.position.y = xy_si[1] + e_y_i * np.cos(psi_si)
            pose.pose.position.z = 0

            msg.poses.append(pose)
        
        self.pub_pred.publish(msg)

        # Post odom

        odom_pose = PoseStamped()
        odom_pose.header.seq = 1
        odom_pose.header.frame_id = 'map'
        odom_pose.header.stamp = rospy.Time.now()
        odom_pose.pose = odom.pose.pose
        self.pub_odom.publish(odom_pose)

    def visualize_state(self, interpolation:Interpolation, currentS, x0: np.ndarray):

        #Publish current position
        x0_pose = PoseStamped()
        xy_s = interpolation.evaluateInterpolation(currentS)
        psi_s = interpolation.getHeadingOfCurveSingle(currentS)

        e_y = x0[0]
        e_psi = x0[1]
        psi = e_psi + psi_s

        quaternion = quaternion_from_euler(0, 0, psi)

        x0_pose.pose.position.x = xy_s[0] - e_y * np.sin(psi_s)
        x0_pose.pose.position.y = xy_s[1] + e_y * np.cos(psi_s)
        x0_pose.pose.position.z = 0
        x0_pose.pose.orientation.x = quaternion[0]
        x0_pose.pose.orientation.y = quaternion[1]
        x0_pose.pose.orientation.z = quaternion[2]
        x0_pose.pose.orientation.w = quaternion[3]

        x0_pose.header.seq = 1
        x0_pose.header.frame_id = 'map'
        x0_pose.header.stamp = rospy.Time.now()

        self.pub_state.publish(x0_pose)

        # Publish current velocity, delta and predicted acceleration
        curr_velocity = Float32() 
        curr_delta = Float32()

        curr_velocity.data = x0[2]
        curr_delta.data = x0[3]

        self.pub_v.publish(curr_velocity)
        self.pub_delta.publish(curr_delta)

    def visualize_normal(self, interpolation:Interpolation):
        msg = PolygonStamped()
        msg.header.seq = 1
        msg.header.frame_id = 'map'
        msg.header.stamp = rospy.Time.now()
        n = 100 # number of evaluation points
        for i in range(n):
            
            
            XY = interpolation.getNormalVectorOfHeadingSingle(i*interpolation.lengthOfCurve/n)
            XY2 = interpolation.evaluateInterpolation(i*interpolation.lengthOfCurve/n)

            point = Point32()
            point.x = XY2[0]
            point.y = XY2[1]
            point.z = 0

            normal = Point32()
            normal.x = XY[0]
            normal.y = XY[1]
            normal.z = 0

            point2 = Point32(point.x+2*normal.x,point.y+2*normal.y,0)
        
            msg.polygon.points.append(point)
            msg.polygon.points.append(point2)
        self.pub_norm.publish(msg)

    def plot_interpolation(self, interpolation: Interpolation, currentXY):

        n = 1000 # number of evaluation points

        XY0 = interpolation.evaluateInterpolation(0)
        currentS = interpolation.getInitialS(currentXY, 0.001)
        currentPsi = interpolation.getHeadingOfCurveSingle(currentS)
        currentKappa = interpolation.getLocalCurvatureOfCurveSingle(currentS)
        currentRho = 1/interpolation.getLocalCurvatureOfCurveSingle(currentS)

        ss = []
        xy = [[],[]]
        kappa = []
        rho = []
        psi = []
        nv = []

        for i in range(n):
            s = i*interpolation.lengthOfCurve/n
            ss.append(s)
            XY = interpolation.evaluateInterpolation(s)
            xy[0].append(XY[0])
            xy[1].append(XY[1])
            psi.append(interpolation.getHeadingOfCurveSingle(s))
            kappa.append(interpolation.getLocalCurvatureOfCurveSingle(s))
            rho.append(1/interpolation.getLocalCurvatureOfCurveSingle(s))
            nv.append(interpolation.getNormalVectorOfHeadingSingle(s))


        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2)
        ax1.plot(xy[0], xy[1])
        ax1.plot(currentXY[0], currentXY[1], marker='o')
        ax1.plot(XY0[0], XY0[1], marker='o')

        for i in range(n):
            ax1.plot([xy[0][i], xy[0][i] + nv[i][0]], [xy[1][i], xy[1][i] + nv[i][1]], )

        ax1.set_title('Track')
        ax1.legend(['Curve', 'Current Position', 'Curve(s = 0)'])
        ax2.plot(ss, psi)
        ax2.plot(currentS, currentPsi, marker='o')
        ax2.set_title('Psi (Heading)')
        ax3.plot(ss, kappa)
        ax3.plot(currentS, currentKappa, marker='o')
        ax3.set_title('Kappa (Curvature)')
        ax4.plot(ss, rho)
        ax4.plot(currentS, currentRho, marker='o')
        ax4.set_title('Rho (Radius)')

        plt.show()
    
    def visualize_obstacle_line(self, points):
        id = self.id
        msg = Marker()
        msg.header.frame_id = 'map'
        msg.header.stamp = rospy.Time.now()
        msg.type = Marker.LINE_STRIP
        msg.action = Marker.ADD
        msg.id = ++id
        msg.scale.x = 0.1
        msg.scale.y = 0.1
        msg.scale.z = 0.1
        msg.color.r = 255
        msg.color.g = 10
        msg.color.b = 10
        msg.color.a = 1.0
        msg.ns = "lines"
        
        pt1 = Point()
        pt2 = Point()
        pt1.x = points[0][0]
        pt1.y = points[0][1]
        pt2.x = points[1][0]
        pt2.y = points[1][1]
        pt1.z = 0
        pt2.z = 0
        
        msg.points.append(pt1)
        msg.points.append(pt2)

        self.pub_obs.publish(msg)

    def visualize_obstacle_poly(self, poly):
        id = self.id
        msg = Marker()
        msg.header.frame_id = 'map'
        msg.header.stamp = rospy.Time.now()
        msg.type = Marker.LINE_STRIP
        msg.action = Marker.ADD
        msg.id = ++id
        msg.scale.x = 0.1
        msg.scale.y = 0.1
        msg.scale.z = 0.1
        msg.color.r = 10
        msg.color.g = 255
        msg.color.b = 10
        msg.color.a = 1.0
        msg.ns = "polys"
        
        for point in poly:
            pt = Point()
            pt.x = point[0]
            pt.y = point[1]
            pt.z = 0
            msg.points.append(pt)

        self.pub_obs.publish(msg)

    def visualize_obstacle_param(self, interpolation:Interpolation, s, e_obs, width):
        id = self.id
        msg = Marker()
        msg.header.frame_id = 'map'
        msg.header.stamp = rospy.Time.now()
        msg.type = Marker.LINE_STRIP
        msg.action = Marker.ADD
        msg.id = ++id
        msg.scale.x = 0.1
        msg.scale.y = 0.1
        msg.scale.z = 0.1
        msg.color.r = 10
        msg.color.g = 10
        msg.color.b = 255
        msg.color.a = 1.0
        msg.ns = "param"
        
        pt1 = Point()
        pt2 = Point()
        pt3 = Point()
        XY = interpolation.evaluateInterpolation(s)
        norm  = interpolation.getNormalVectorOfHeadingSingle(s)

        pt1.x = XY[0] + e_obs * norm[0]
        pt1.y = XY[1] + e_obs * norm[1]

        pt1.z = 0

        pt2.x = XY[0] + (norm[0] * (e_obs + width/2))
        pt2.y = XY[1] + (norm[1] * (e_obs + width/2))
        pt2.z = 0
        pt3.x = XY[0] + (norm[0] * (e_obs - width/2))
        pt3.y = XY[1] + (norm[1] * (e_obs - width/2))
        pt3.z = 0

        msg.points.append(pt1)
        msg.points.append(pt2)
        msg.points.append(pt3)

        self.pub_obs.publish(msg)
        
