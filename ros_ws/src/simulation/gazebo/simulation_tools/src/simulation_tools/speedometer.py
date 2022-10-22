#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState, ModelStates, LinkState, LinkStates

from track import track, Point

import time

wheel_velocity = None
car_velocity = None
max_car_velocity = 0

car_acceleration = None
max_car_acceleration = 0
min_car_acceleration = 0
car_velocity_old = None
time_old = None

model_states_message = None
link_states_message = None


def model_state_callback(message):
    global model_states_message
    model_states_message = message


def link_state_callback(message):
    global link_states_message
    link_states_message = message


def calculate_car_velocity():
    global max_car_velocity, car_velocity
    if len(model_states_message.pose) < 2:
        return
    car_velocity = (model_states_message.twist[1].linear.x**2 +
                    model_states_message.twist[1].linear.y**2)**0.5
    if car_velocity > max_car_velocity:
        max_car_velocity = car_velocity


LINK_NAMES = [
    'racer::left_wheel_front',
    'racer::left_wheel_back',
    'racer::right_wheel_front',
    'racer::right_wheel_back']
WHEEL_RADIUS = 0.05


def calculate_wheel_velocity():
    global wheel_velocity
    indices = [i for i in range(len(link_states_message.name))
               if link_states_message.name[i] in LINK_NAMES]
    twists = [link_states_message.twist[i].angular for i in indices]

    angle_velocities = [(t.x**2 + t.y**2)**0.5 for t in twists]
    angular_velocity = sum(angle_velocities) / len(angle_velocities)
    wheel_velocity = angular_velocity * WHEEL_RADIUS


def show_info():
    if car_velocity is None or max_car_velocity is None or wheel_velocity is None or car_acceleration is None:
        return

    position = Point(
        model_states_message.pose[1].position.x,
        model_states_message.pose[1].position.y)

    rospy.loginfo(
        "car: {0:.2f} m/s, max: {1:.2f} m/s, wheels: {2:.2f} m/s, slip:".format(
            car_velocity,
            max_car_velocity,
            wheel_velocity) +
        "{0:.2f}\n".format(
            wheel_velocity -
            car_velocity).rjust(7) +
        "accel: {0: 2f}m/s^2, max: {1: 2f}m/s^2, min: {2: 2f}m/s^2\n".format(
            car_acceleration,
            max_car_acceleration,
            min_car_acceleration) +
        str(
                track.localize(position)) +
        ", world: ({0:.2f}, {1:.2f})\n".format(
            position.x,
            position.y) +
        "-------------------------------------------")


idle = True


def calculate_velocity(event):
    global idle
    if model_states_message is None or link_states_message is None:
        return

    calculate_car_velocity()
    calculate_wheel_velocity()
    calculate_acceleration()

    if abs(car_velocity) < 0.01 and abs(wheel_velocity) < 0.01:
        if idle:
            return
        else:
            idle = True
    show_info()


def calculate_acceleration():
    global car_acceleration, max_car_acceleration, min_car_acceleration, car_velocity_old, time_old
    time_now = rospy.Time.now()
    if car_velocity is not None and car_velocity_old is not None and time_old is not None:
        delta_time = time_now - time_old
        if delta_time.to_nsec() != 0:
            car_acceleration = (
                car_velocity - car_velocity_old) / delta_time.to_sec()
    time_old = time_now
    car_velocity_old = car_velocity
    if car_acceleration is not None and car_acceleration > max_car_acceleration:
        max_car_acceleration = car_acceleration
    elif car_acceleration is not None and car_acceleration < min_car_acceleration:
        min_car_acceleration = car_acceleration


rospy.init_node('speedometer', anonymous=True)
rospy.Subscriber("/gazebo/model_states", ModelStates, model_state_callback)
rospy.Subscriber("/gazebo/link_states", LinkStates, link_state_callback)
rospy.Timer(rospy.Duration(0.05, 0), calculate_velocity)

rospy.spin()
