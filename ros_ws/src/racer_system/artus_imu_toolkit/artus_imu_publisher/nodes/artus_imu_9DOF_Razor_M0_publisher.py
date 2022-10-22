#!/usr/bin/env python

## @package artus_imu_publisher
# @file        artus_imu_9DOF_Razor_M0_publisher.py
# @brief       ROS Publisher for 9DOF_Razor_M0 IMU Data exchanged between an MCU (eg. Arduino) via Serial(-over-USB)
# @mainpage    ARTUS IMU TOOLKIT
# @author      Samanta Scharmacher <samanta.scharmacher@tu-dortmund.de>
#
# @note        This implementation is inspired by the Razor_AHRS ROS Package implementation (Copyright (c) 2012, Tang Tiong Yew)

# Copyright (c) 2022, Samanta Scharmacher
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

### TODO: Implement print calibration data stored on MCU
### TODO: Implement sending new CalibData while IMU is running... will there be a problem with filter if filter input will change unexpectedly?

import rospy
import serial
import string
import sys

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion , Vector3Stamped #, PoseStamped ##TODO: Think about how to calculate initial pose? should we start at 0,0,0? or should we have a look at other position topics and use them as a reference (at least for init-time?)?
from dynamic_reconfigure.server import Server
from artus_imu_sensors.cfg import artus_imu_sensorsConfig

# Callback for dynamic reconfigure requests
def reconfig_callback(config, level):
    #global imu_yaw_calibration
    rospy.loginfo("""Reconfigure Request for  IMU-Sensor: Sensor_Name: {imu_sensor_name} ; Serial_Port: {serial_port} ; \n  
    Calibration_Data: _NO_DYNAMIC_SENSOR_CALIBRATION_UPDATE_AT_THE_MOMENT_ ;  \n  
    Orientation_Covariance_Matrix: \n{orientation_cov_00} , {orientation_cov_01} , {orientation_cov_02} ;\n{orientation_cov_10} , {orientation_cov_11} , {orientation_cov_12} ; \n{orientation_cov_20} , {orientation_cov_21} , {orientation_cov_22}; \n 
    AngularVelocity_Covariance_Matrix: \n{angularVelocity_cov_00} , {angularVelocity_cov_01} , {angularVelocity_cov_02} ;\n{angularVelocity_cov_10} , {angularVelocity_cov_11} , {angularVelocity_cov_12} ; \n{angularVelocity_cov_20} , {angularVelocity_cov_21} , {angularVelocity_cov_22}; \n 
    LinearAcceleration_Covariance_Matrix: \n{linearAcc_cov_00} , {linearAcc_cov_01} , {linearAcc_cov_02} ;\n{linearAcc_cov_10} , {linearAcc_cov_11} , {linearAcc_cov_12} ; \n{linearAcc_cov_20} , {linearAcc_cov_21} , {linearAcc_cov_22}; \n 
    """ .format(**config))
    return config

# TODO: Maybe use param_server to set node-name "dynamically"... Is it possible here?
rospy.init_node("artus_imu_9DOF_Razor_M0_pub_py_node")
srv = Server(artus_imu_sensorsConfig, reconfig_callback)  # define dynamic_reconfigure callback

####### ~~~ Pre-Initializing Informations/Data ~~~ ###########
# --- checking if config-yaml file is present with sensor data instead of using hardcoded default-values... --- #
### TODO: Is it necessary when using default Values?
yamlConfig_exists = False
imu_sensor_name = '__IMU_SENSOR_NAME_NOT_SET__'
if rospy.has_param('~imu_sensor_name'):
    imu_sensor_name = rospy.get_param('~imu_sensor_name' , '__IMU_SENSOR_NAME_NOT_SET__')
    yamlConfig_exists = True

### Initialize Calibration Data for every single Sensor-Module on the IMU ###
### TODO: Dynamically updating Process while the IMU-sensor is still running isn't implemented jet.. All Calibration Data is hard coded in Arduino firmware at the moment. But usefull methods for updating these information are already implemented
# TODO!!!

### Initialize Covariance Data for every single IMU-Message Section of the IMU ###
# Load that information from config-yaml file to easily adjust, reuse and swap covariance configuration/information
# pre-initialize variable with default values in case no config-yaml exists or reading from it would fail...

# ~ initialize orientation_covariance ~ #
orientation_covariance = [
    rospy.get_param('~orientation_cov_00' , 0.0025) , rospy.get_param('~orientation_cov_01' , 0.0) , rospy.get_param('~orientation_cov_02' , 0.0),
    rospy.get_param('~orientation_cov_10' , 0.0), rospy.get_param('~orientation_cov_11' , 0.0025), rospy.get_param('~orientation_cov_12' , 0.0),
    rospy.get_param('~orientation_cov_20' , 0.0), rospy.get_param('~orientation_cov_21' , 0.0), rospy.get_param('~orientation_cov_22' , 0.0025)
]

# ~ initialize angular_velocity_covariance ~ #
angular_velocity_covariance = [
    rospy.get_param('~angularVelocity_cov_00' , 0.02) , rospy.get_param('~angularVelocity_cov_01' , 0.0) , rospy.get_param('~angularVelocity_cov_02' , 0.0),
    rospy.get_param('~angularVelocity_cov_10' , 0.0), rospy.get_param('~angularVelocity_cov_11' , 0.02), rospy.get_param('~angularVelocity_cov_12' , 0.0),
    rospy.get_param('~angularVelocity_cov_20' , 0.0), rospy.get_param('~angularVelocity_cov_21' , 0.0), rospy.get_param('~angularVelocity_cov_22' , 0.02)
]

# ~ initialize linear_acceleration_covariance ~ #
linear_acceleration_covariance = [
    rospy.get_param('~linearAcc_cov_00' , 0.04) , rospy.get_param('~linearAcc_cov_01' , 0.0) , rospy.get_param('~linearAcc_cov_02' , 0.0),
    rospy.get_param('~linearAcc_cov_10' , 0.0), rospy.get_param('~linearAcc_cov_11' , 0.04), rospy.get_param('~linearAcc_cov_12' , 0.0),
    rospy.get_param('~linearAcc_cov_20' , 0.0), rospy.get_param('~linearAcc_cov_21' , 0.0), rospy.get_param('~linearAcc_cov_22' , 0.04)
]

port = rospy.get_param('~serial_port', '/dev/ttyACM1')

# Check your ttyACM port and baud rate
rospy.loginfo("For %s opening SerialPort: %s...", imu_sensor_name ,  port)
try:
    ser = serial.Serial(port=port, baudrate=115200, timeout=1)
except serial.serialutil.SerialException:
    rospy.logerr( imu_sensor_name + " IMU Sensor not found at port "+port + ". Did you specify the correct port in the launch file?")
    #exit
    sys.exit(0)

#read/write calibration parameters
## TODO: implement a bidirectional Communication for eg. exchanging calibration Values... without HardCoding them... take the original razor 9dof Package as reference...

####### ~~~ Initializing Messaging & Message-Objects ~~~ ###########
# ~~~ Initialize topic  publisher ~~~ #
# We only care about the most recent measurement, i.e. queue_size=1
pub_imu = rospy.Publisher( 'imu_' + imu_sensor_name , Imu, queue_size=1)
pub_relVelocity = rospy.Publisher( 'imu_' + imu_sensor_name + '/relativeVelocity' , Vector3Stamped, queue_size=1)
pub_relDistance = rospy.Publisher( 'imu_' + imu_sensor_name + '/relativeDistance' , Vector3Stamped, queue_size=1)
# ToDo: ##pub_imuPose = rospy.Publisher( 'imu_' + imu_sensor_name + '/pose' , PoseStamped, queue_size=1)

### general imu message content pre-initializing ###
seq = 0   # initialize message sequential number
imu_frame_id = 'base_imu_link'
imu_timeStamp = rospy.Time.now()

### ROS standard imu message pre-initializing ###
imuMsg = Imu()
imuMsg.orientation_covariance = orientation_covariance
imuMsg.angular_velocity_covariance = angular_velocity_covariance
imuMsg.linear_acceleration_covariance = linear_acceleration_covariance

### Additional imu message for ROS pre-initializing ###
### TODO: Adjust other infomation-Message Types or find matching standard alternative

relVelocityMsg = Vector3Stamped()
relDistanceMsg = Vector3Stamped()


####### ~~~ Initializing IMU Board ~~~ ###########
rospy.loginfo("Giving the " + imu_sensor_name + " IMU board 1 second to boot...")
rospy.sleep(1) # Sleep for 5 seconds to wait for the board to boot

### TODO: Dynamically updating Process while the IMU-sensor is still running isn't implemented jet.. All Calibration Data is hard coded in Arduino firmware at the moment. But usefull methods for updating these information are already implemented
# TODO!!!

### configure board ###
# -stop datastream
# -discard old input
# -automatic flush - NOT WORKING
# -flush manually, as above command is not working
# -rospy.loginfo("Writing calibration values to razor IMU board...")
# -set calibration values
# -print calibration values for verification by user
# -re-start datastream

# -automatic flush - NOT WORKING
# -ser.flushInput()  #discard old input, still in invalid format
# -flush manually, as above command is not working - it breaks the serial connection

# TODO: How many messages have to be removed/flushed actually
rospy.loginfo("Flushing first 200 IMU entries...")
for x in range(0, 200):
    line = ser.readline()
rospy.loginfo("Publishing IMU data...")

### ==== enter main_loop === #
while not rospy.is_shutdown():
    line = ser.readline()
    rospy.loginfo(line)
    # Skip if a hiccup happened
    # TODO: using regex
    if line[0] != "#" or line[1] != "Q" :
        rospy.logerr("No Matching start pattern in message")
        continue
    if line[2:].find('#Q') != -1 :
        rospy.logerr("Too many start pattern in message")
        continue
    line = line.replace("#Q","")        # Delete "#Q "
    words = string.split(line,",")      # Fields split
    if len(words) > 16 :
        rospy.logerr("Too many elements were extracted from message!")
        continue
    if len(words) < 16 :
        rospy.logerr("Too less elements were extracted from message!")
        continue

    imu_timeStamp = rospy.Time.now()

    q = Quaternion(float(words[0]) , float(words[1]), float(words[2]), float(words[3]))
    imuMsg.orientation.x = q[0]
    imuMsg.orientation.y = q[1]
    imuMsg.orientation.z = q[2]
    imuMsg.orientation.w = q[3]

    imuMsg.linear_acceleration.x = float(words[4])
    imuMsg.linear_acceleration.y = float(words[5])
    imuMsg.linear_acceleration.z = float(words[6])

    imuMsg.angular_velocity.x = float(words[7])
    imuMsg.angular_velocity.y = float(words[8])
    imuMsg.angular_velocity.z = float(words[9])

    imuMsg.header.stamp= imu_timeStamp   # rospy.Time.now()
    imuMsg.header.frame_id = imu_frame_id # 'base_imu_link'
    imuMsg.header.seq = seq

    relVelocityMsg.Vector3.x = float(words[10])
    relVelocityMsg.Vector3.y = float(words[11])
    relVelocityMsg.Vector3.z = float(words[12])
    relVelocityMsg.header.stamp = imu_timeStamp  # rospy.Time.now()
    relVelocityMsg.header.frame_id = imu_frame_id # 'base_imu_link'
    relVelocityMsg.header.seq = seq


    relDistanceMsg.Vector3.x = float(words[13])
    relDistanceMsg.Vector3.y = float(words[14])
    relDistanceMsg.Vector3.z = float(words[15])
    relDistanceMsg.header.stamp = imu_timeStamp ## rospy.Time.now()
    relDistanceMsg.header.frame_id = imu_frame_id # 'base_imu_link'
    relDistanceMsg.header.seq = seq

    pub_imu.publish(imuMsg)
    pub_relVelocity.publish(relVelocityMsg)
    pub_relDistance.publish(relDistanceMsg)

    seq = seq + 1

ser.close
