#!/usr/bin/env python3.8

from operator import truediv
import rospy
from f1tenth_gym_agent.msg import Observation, drive_param
from eyes_msgs.msg import PoseWithScan
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import time
import rosgraph
import socket
import numpy as np

from tf2_ros import transform_broadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class ROSRunner:
    def __init__(self, agent_name, opp_name):
        self.agent_name = agent_name
        self.opp_name = opp_name
        self.pub_drive = None
        self.pub_odom = None
        self.roscore_started = False
        self.last_healthy_time = None
        self.pub_tf = None

        self.ego_motion_error_x = 0.0
        self.ego_motion_error_y = 0.0
        self.ego_orientation_error = 0.0

        self.opp_motion_error_x = 0.0
        self.opp_motion_error_y = 0.0
        self.opp_orientation_error = 0.0

        self.ego_init = True
        self.opp_init = True

        # Scan simulation params
        scan_fov = rospy.get_param('scan_fov')
        scan_beams = rospy.get_param('scan_beams')
        self.angle_min = -scan_fov / 2.
        self.angle_max = scan_fov / 2.
        self.angle_inc = scan_fov / scan_beams


    def drive_callback(self, data):
        self.last_healthy_time = time.time()
        # create message & publish
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.drive.speed = data.velocity
        msg.drive.steering_angle = data.angle
        self.pub_drive.publish(msg)

        
    def drive_callback_opp(self, data):
        self.last_healthy_time = time.time()
        # create message & publish
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.drive.speed = data.velocity
        msg.drive.steering_angle = data.angle
        self.pub_drive_opp.publish(msg)


    def observation_callback(self, data):
        pose_x = 0.0
        pose_y = 0.0
        orientation = 0.0

        self.last_healthy_time = time.time()
        ego_pose = data.ego_pose
        ego_twist = data.ego_twist

        ground_truth_pose = PoseWithCovarianceStamped()
        ground_truth_pose.header.stamp = rospy.Time.now()
        ground_truth_pose.header.frame_id = "/map"
        ground_truth_pose.pose = ego_pose
        
        self.pub_ground_truth_pose.publish(ground_truth_pose)

        if self.ego_init:
            self.ego_init = False
            self.pub_initial_pose.publish(ground_truth_pose)

        if not self.ground_truth_localization:
            # self.ego_motion_error_x += np.random.normal() * 0.00005 + ego_twist.twist.linear.x * 0.0005 * np.random.normal() + ego_twist.twist.angular.z * 0.0005 * np.random.normal()
            # self.ego_motion_error_y += np.random.normal() * 0.00001 + ego_twist.twist.linear.y * 0.0001 * np.random.normal() + ego_twist.twist.angular.z * 0.00025 * np.random.normal()
            # self.ego_orientation_error += np.random.normal() * 0.00002 + ego_twist.twist.angular.z * 0.00055 * np.random.normal()

            pose_x = ego_pose.pose.position.x + self.ego_motion_error_x
            pose_y = ego_pose.pose.position.y + self.ego_motion_error_y
            explicit_quat = [ego_pose.pose.orientation.x, ego_pose.pose.orientation.y, ego_pose.pose.orientation.z, ego_pose.pose.orientation.w]
            euler = euler_from_quaternion(explicit_quat)[2] + self.ego_orientation_error
            quat = quaternion_from_euler(0.0, 0.0, euler)
            orientation = Quaternion()
            orientation.x = quat[0]
            orientation.y = quat[1]
            orientation.z = quat[2]
            orientation.w = quat[3]

            noise_pose_msg = PoseWithCovarianceStamped()
            noise_pose_msg.header.stamp = rospy.Time.now()
            noise_pose_msg.header.frame_id = "map"
            noise_pose_msg.pose.pose.position.x = pose_x
            noise_pose_msg.pose.pose.position.y = pose_y
            noise_pose_msg.pose.pose.orientation = orientation

            self.pub_noisy_pose.publish(noise_pose_msg)
        else:
            pose_x = ego_pose.pose.position.x
            pose_y = ego_pose.pose.position.y
            orientation = ego_pose.pose.orientation

        # create message & publish
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        # msg.header.frame_id = "ego_racecar/odom"
        msg.header.frame_id = "map"
        msg.child_frame_id = "ego_racecar/base_link"
        msg.pose = ego_pose
        msg.pose.pose.position.x = pose_x
        msg.pose.pose.position.y = pose_y
        msg.pose.pose.orientation = orientation
        msg.twist = ego_twist

        self.pub_odom.publish(msg)

        gt_t = Transform()
        gt_t.translation.x = ego_pose.pose.position.x
        gt_t.translation.y = ego_pose.pose.position.y
        gt_t.translation.z = 0.0
        gt_t.rotation = msg.pose.pose.orientation

        gt_ts = TransformStamped()
        gt_ts.transform = gt_t
        gt_ts.header.stamp = msg.header.stamp
        gt_ts.header.frame_id = 'map'
        gt_ts.child_frame_id = 'ego_racecar/gt'
        
        self.pub_tf.sendTransform(gt_ts)

        if self.publish_odom_tf:
            # publish odometry tf
            ego_t = Transform()
            ego_t.translation.x = pose_x
            ego_t.translation.y = pose_y
            ego_t.translation.z = 0.0
            ego_t.rotation = msg.pose.pose.orientation # + np.random.normal() * 0.29

            ego_ts = TransformStamped()
            ego_ts.transform = ego_t
            ego_ts.header.stamp = msg.header.stamp
            ego_ts.header.frame_id = 'ego_racecar/odom'
            ego_ts.child_frame_id = 'ego_racecar/base_link'
            
            self.pub_tf.sendTransform(ego_ts)

        if self.ground_truth_localization:
            scan = LaserScan()
            scan.header.stamp = msg.header.stamp
            scan.header.frame_id = "ego_racecar/base_link"
            scan.angle_min = self.angle_min
            scan.angle_max = self.angle_max
            scan.angle_increment = self.angle_inc
            scan.range_min = 0.
            scan.range_max = 15.
            scan.ranges = data.ranges

            # msg_odom_scan = OdomScan()
            # msg_odom_scan.scan = scan
            # msg_odom_scan.odom = msg

            # self.pub_odom_scan.publish(msg_odom_scan)

            pose_with_scan = PoseWithScan()
            pose_with_scan.scan = scan
            pose_with_scan.pose = ego_pose

            self.pub_pose_with_scan.publish(pose_with_scan)

            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "map"
            pose_msg.pose = ego_pose
            
            self.pub_pose.publish(pose_msg)

            # publish map tf
            ego_t = Transform()
            ego_t.translation.x = 0.0
            ego_t.translation.y = 0.0
            ego_t.translation.z = 0.0
            quat = quaternion_from_euler(0.0, 0.0, 0.0)
            ego_t.rotation.x = quat[0]
            ego_t.rotation.y = quat[1]
            ego_t.rotation.z = quat[2]
            ego_t.rotation.w = quat[3]

            ego_ts = TransformStamped()
            ego_ts.transform = ego_t
            ego_ts.header.stamp = msg.header.stamp
            ego_ts.header.frame_id = 'map'
            ego_ts.child_frame_id = 'ego_racecar/odom'      

            self.pub_tf.sendTransform(ego_ts)

    def observation_callback_opp(self, data):
        pose_x = 0.0
        pose_y = 0.0
        orientation = 0.0

        self.last_healthy_time = time.time()
        opp_pose = data.ego_pose
        opp_twist = data.ego_twist

        ground_truth_pose = PoseWithCovarianceStamped()
        ground_truth_pose.header.stamp = rospy.Time.now()
        ground_truth_pose.header.frame_id = "map"
        ground_truth_pose.pose = opp_pose
        
        self.pub_ground_truth_pose_opp.publish(ground_truth_pose)

        # if self.opp_init:
        #     self.opp_init = False
        #     self.pub_initial_pose_opp.publish(ground_truth_pose)

        # if not self.ground_truth_localization:
        #     # self.opp_motion_error_x += np.random.normal() * 0.0003 + opp_twist.twist.linear.x * 0.004 * np.random.normal() + opp_twist.twist.angular.x * 0.0025 * np.random.normal()
        #     # self.opp_motion_error_x += np.random.normal() * 0.0002 + opp_twist.twist.linear.y * 0.0035 * np.random.normal() + opp_twist.twist.angular.y * 0.004 * np.random.normal()
        #     # self.opp_orientation_error += np.random.normal() * 0.001

        #     pose_x = opp_pose.pose.position.x + self.opp_motion_error_x
        #     pose_y = opp_pose.pose.position.y + self.opp_motion_error_x
        #     explicit_quat = [opp_pose.pose.orientation.x, opp_pose.pose.orientation.y, opp_pose.pose.orientation.z, opp_pose.pose.orientation.w]
        #     euler = euler_from_quaternion(explicit_quat)[2] + self.opp_orientation_error
        #     quat = quaternion_from_euler(0.0, 0.0, euler)
        #     orientation = Quaternion()
        #     orientation.x = quat[0]
        #     orientation.y = quat[1]
        #     orientation.z = quat[2]
        #     orientation.w = quat[3]

        #     noise_pose_msg = PoseWithCovarianceStamped()
        #     noise_pose_msg.header.stamp = rospy.Time.now()
        #     noise_pose_msg.header.frame_id = "map"
        #     noise_pose_msg.pose.pose.position.x = pose_x
        #     noise_pose_msg.pose.pose.position.y = pose_y
        #     noise_pose_msg.pose.pose.orientation = orientation

        #     self.pub_noisy_pose_opp.publish(noise_pose_msg)
        # else:
        pose_x = opp_pose.pose.position.x
        pose_y = opp_pose.pose.position.y
        orientation = opp_pose.pose.orientation
        
        # create message & publish
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "opp_racecar/base_link"
        msg.pose = opp_pose
        msg.pose.pose.position.x = pose_x
        msg.pose.pose.position.y = pose_y
        msg.pose.pose.orientation = orientation
        msg.twist = opp_twist

        self.pub_odom_opp.publish(msg)

        # gt_t = Transform()
        # gt_t.translation.x = opp_pose.pose.position.x
        # gt_t.translation.y = opp_pose.pose.position.y
        # gt_t.translation.z = 0.0
        # gt_t.rotation = msg.pose.pose.orientation

        # gt_ts = TransformStamped()
        # gt_ts.transform = gt_t
        # gt_ts.header.stamp = msg.header.stamp
        # gt_ts.header.frame_id = 'map'
        # gt_ts.child_frame_id = 'opp_racecar/gt'
        
        # self.pub_tf_opp.sendTransform(gt_ts)

        # if self.publish_odom_tf:
            # publish odometry tf
        opp_t = Transform()
        opp_t.translation.x = pose_x
        opp_t.translation.y = pose_y
        opp_t.translation.z = 0.0
        opp_t.rotation = msg.pose.pose.orientation

        opp_ts = TransformStamped()
        opp_ts.transform = opp_t
        opp_ts.header.stamp = msg.header.stamp
        opp_ts.header.frame_id = 'opp_racecar/odom'
        opp_ts.child_frame_id = 'opp_racecar/base_link'
        
        self.pub_tf_opp.sendTransform(opp_ts)

        # if self.ground_truth_localization:
            # scan = LaserScan()
            # scan.header.stamp = msg.header.stamp
            # scan.header.frame_id = "opp_racecar/base_link"
            # scan.angle_min = self.angle_min
            # scan.angle_max = self.angle_max
            # scan.angle_increment = self.angle_inc
            # scan.range_min = 0.
            # scan.range_max = 15.
            # scan.ranges = data.ranges

            # # msg_odom_scan = OdomScan()
            # # msg_odom_scan.scan = scan
            # # msg_odom_scan.odom = msg

            # # self.pub_odom_scan_opp.publish(msg_odom_scan)

            # pose_with_scan = PoseWithScan()
            # pose_with_scan.scan = scan
            # pose_with_scan.pose = opp_pose

            # self.pub_pose_with_scan_opp.publish(pose_with_scan)

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.pose = opp_pose
        
        self.pub_pose_opp.publish(pose_msg)

        # publish map tf
        opp_t = Transform()
        opp_t.translation.x = 0.0
        opp_t.translation.y = 0.0
        opp_t.translation.z = 0.0
        quat = quaternion_from_euler(0.0, 0.0, 0.0)
        opp_t.rotation.x = quat[0]
        opp_t.rotation.y = quat[1]
        opp_t.rotation.z = quat[2]
        opp_t.rotation.w = quat[3]

        opp_ts = TransformStamped()
        opp_ts.transform = opp_t
        opp_ts.header.stamp = msg.header.stamp
        opp_ts.header.frame_id = 'map'
        opp_ts.child_frame_id = 'opp_racecar/odom'
        
        self.pub_tf_opp.sendTransform(opp_ts)

    def laser_callback(self, msg):
        self.last_healthy_time = time.time()
        laser = msg
        # laser.header.stamp = rospy.Time.now()
        laser.header.frame_id = "ego_racecar/base_link" # "ego_racecar/laser_model"
        laser.range_max = 15
        self.pub_lidar.publish(laser)
        
    def laser_callback_opp(self, msg):
        self.last_healthy_time = time.time()
        laser = msg
        laser.header.stamp = rospy.Time.now()
        laser.header.frame_id = "opp_racecar/base_link" # "opp_racecar/laser_model"
        laser.range_max = 15
        self.pub_lidar_opp.publish(laser)

        self.last_opp_scan = laser


    def run(self):
        rospy.init_node('gym_agent_%s' % self.agent_name, anonymous=True)

        # start publishing
        self.pub_drive = rospy.Publisher(
            '/%s/drive' % self.agent_name, AckermannDriveStamped, queue_size=5)
        
        self.pub_drive_opp = rospy.Publisher(
            '/%s/drive' % self.opp_name, AckermannDriveStamped, queue_size=5)

        self.pub_lidar = rospy.Publisher('/%s/lidar' % self.agent_name, LaserScan, queue_size=5)
        self.pub_lidar_opp = rospy.Publisher('/%s/lidar' % self.opp_name, LaserScan, queue_size=5)
        
        #TODO prepend ego name before /odom topic
        self.pub_odom = rospy.Publisher('%s/odom' % self.agent_name, Odometry, queue_size=5)
        self.pub_odom_opp = rospy.Publisher('/%s/odom' % self.opp_name, Odometry, queue_size=5)

        self.pub_ground_truth_pose = rospy.Publisher('/%s/gt_pose' % self.agent_name, PoseWithCovarianceStamped, queue_size=5)
        self.pub_ground_truth_pose_opp = rospy.Publisher('/%s/gt_pose' % self.opp_name, PoseWithCovarianceStamped, queue_size=5)

        # self.pub_odom_scan = rospy.Publisher('%s/odom_scan' % self.agent_name, OdomScan, queue_size=5)
        # self.pub_odom_scan_opp = rospy.Publisher('/%s/odom_scan' % self.opp_name, OdomScan, queue_size=5)

        self.pub_initial_pose = rospy.Publisher('%s/initialpose' % self.agent_name, PoseWithCovarianceStamped, queue_size=1)
        self.pub_initial_pose_opp = rospy.Publisher('%s/initialpose' % self.opp_name, PoseWithCovarianceStamped, queue_size=1)

        self.publish_odom_tf = rospy.get_param("/gym_agent/publish_odom_tf", True)

        self.ground_truth_localization = rospy.get_param("/gym_agent/ground_truth_localization", True)

        # always publish these for the opponent
        self.pub_pose_opp = rospy.Publisher('%s/pose' % self.opp_name, PoseWithCovarianceStamped, queue_size=5)
        self.pub_pose_with_scan_opp = rospy.Publisher('%s/pose_with_scan' % self.opp_name, PoseWithScan, queue_size=5)

        if self.ground_truth_localization:
            self.pub_pose_with_scan = rospy.Publisher('%s/pose_with_scan' % self.agent_name, PoseWithScan, queue_size=5)
            self.pub_pose = rospy.Publisher('%s/pose' % self.agent_name, PoseWithCovarianceStamped, queue_size=5)
        else:
            self.pub_noisy_pose = rospy.Publisher('%s/noisy_pose' % self.agent_name, PoseWithCovarianceStamped, queue_size=5)
            self.pub_noisy_pose_opp = rospy.Publisher('%s/noisy_pose' % self.opp_name, PoseWithCovarianceStamped, queue_size=5)
        
        self.pub_tf = transform_broadcaster.TransformBroadcaster()
        self.pub_tf_opp = transform_broadcaster.TransformBroadcaster()

        # start listening
        rospy.Subscriber("/%s/input/drive_param/autonomous" % self.opp_name,
                         drive_param, self.drive_callback_opp)
        
        rospy.Subscriber("/%s/input/drive_param/autonomous" % self.agent_name,
                         drive_param, self.drive_callback)
        
        rospy.Subscriber('/%s/observations' % self.agent_name,
                         Observation, self.observation_callback)
        
        rospy.Subscriber('/%s/observations' % self.opp_name,
                        Observation, self.observation_callback_opp)
        
        rospy.Subscriber('/%s/scan' % self.agent_name,
                         LaserScan, self.laser_callback) 
               
        rospy.Subscriber('/%s/scan' % self.opp_name,
                         LaserScan, self.laser_callback_opp)

        # delay to stop the agent from driving
        # before startup has finished
        delay = rospy.get_param("/gym_agent/init_delay", 5)
        rospy.sleep(delay)

        # Initial start message example
        msg = drive_param()
        msg.velocity = 0.01
        msg.angle = 0
        self.drive_callback(msg)
        self.drive_callback_opp(msg)

        while not rospy.core.is_shutdown():
            current_time = time.time()

            # if it passed more than 5 seconds since last sensor
            # try to connect to ROS, if you cannot
            if self.last_healthy_time and current_time > self.last_healthy_time + 5:
                try:
                    rosgraph.Master('/rostopic').getPid()
                except socket.error:
                    print("ROS is shutdown, shutting down agent node")
                    return
                else:
                    print(
                        "ROS is running, but agent didn't receive observation for a long time. ")
                    print(
                        "You can shutdown your agent docker container if you want to.")
                    self.last_healthy_time = time.time()

            rospy.rostime.wallsleep(0.04)


if __name__ == "__main__":
    # agent_name = os.environ.get("F1TENTH_AGENT_NAME")
    agent_name = rospy.get_param("/gym_agent/ego_id")
    opp_name = rospy.get_param("/gym_agent/opp_id")

    runner = ROSRunner(agent_name, opp_name)

    print("Starting Agent: %s" % agent_name)

    # launch
    runner.run()

    print("Agent run finished")
