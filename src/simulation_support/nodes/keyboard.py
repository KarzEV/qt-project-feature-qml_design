#!/usr/bin/env python

import atexit
import os
import rospy
import sys
import termios
import math
import numpy

from gazebo_msgs.msg import ModelState
from geometry_msgs import msg
from std_msgs.msg import String
import tf.transformations

OLD_SETTINGS = None
RATE = 50


def init_anykey():
    global OLD_SETTINGS
    OLD_SETTINGS = termios.tcgetattr(sys.stdin)

    new_settings = termios.tcgetattr(sys.stdin)
    new_settings[3] = new_settings[3] & ~(termios.ECHO | termios.ICANON)
    new_settings[6][termios.VMIN] = 0
    new_settings[6][termios.VTIME] = 0

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, new_settings)


@atexit.register
def term_anykey():
    global OLD_SETTINGS
    if OLD_SETTINGS is not None:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, OLD_SETTINGS)


def anykey():
    ch_set = []
    ch = os.read(sys.stdin.fileno(), 1)
    while ch is not None and len(ch) > 0:
        ch_set.append(chr(ord(ch[0])))
        ch = os.read(sys.stdin.fileno(), 1)
    if len(ch_set) == 0:
        return ""

    return ch_set[0]


def limit_num(value, min, max):
    if value > max:
        return max
    if value < min:
        return min
    return value


def quaternion_from_numpy(numpy_quaternion):
    ros_quaternion = msg.Quaternion()
    ros_quaternion.x = numpy_quaternion[0]
    ros_quaternion.y = numpy_quaternion[1]
    ros_quaternion.z = numpy_quaternion[2]
    ros_quaternion.w = numpy_quaternion[3]

    return ros_quaternion


def numpy_array_from_quaternion(ros_quaternion):
    return numpy.array([ros_quaternion.x, ros_quaternion.y, ros_quaternion.z, ros_quaternion.w])


class TeleopController(object):
    def __init__(self):
        self.rate = rospy.Rate(RATE)

        self.set_model_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.debug_pub = rospy.Publisher("/test", String, queue_size=1)

        self.linear_step = rospy.get_param("linear_step", 0.1)
        self.rotation_step = rospy.get_param("rotation_step", 0.087)
        self.__init_pose()

    def __init_pose(self):
        self.model_state = ModelState()
        self.model_state.model_name = "URDF_model"
        self.model_state.pose.position.x = rospy.get_param("x", 0.0)
        self.model_state.pose.position.y = rospy.get_param("y", 0.0)
        self.model_state.pose.position.z = rospy.get_param("z", 0.0)

        roll = rospy.get_param("roll", 0.0)
        pitch = rospy.get_param("pitch", 0.0)
        yaw = rospy.get_param("yaw", 0.0)

        self.model_state.pose.orientation = quaternion_from_numpy(
            tf.transformations.quaternion_from_euler(roll, pitch, yaw))

    def update_yaw(self, positive):
        euler = tf.transformations.euler_from_quaternion(numpy_array_from_quaternion(self.model_state.pose.orientation))

        yaw = euler[2]
        if positive:
            yaw += self.rotation_step
        else:
            yaw -= self.rotation_step

        pith = limit_num(yaw, -math.pi, math.pi)
        self.model_state.pose.orientation = quaternion_from_numpy(
            tf.transformations.quaternion_from_euler(euler[0], euler[1], yaw))

    def spin(self, key):
        if key == "w":
            self.model_state.pose.position.x += self.linear_step
        if key == "a":
            self.model_state.pose.position.y += self.linear_step
        if key == "s":
            self.model_state.pose.position.x -= self.linear_step
        if key == "d":
            self.model_state.pose.position.y -= self.linear_step
        if key == "q":
            self.update_yaw(True)
        if key == "e":
            self.update_yaw(False)
        if key == "z":
            self.model_state.pose.position.z += self.linear_step
        if key == "x":
            self.model_state.pose.position.z -= self.linear_step

        self.set_model_pub.publish(self.model_state)
        self.debug_pub.publish(str(self.model_state))
        self.rate.sleep()


def main():
    init_anykey()
    try:
        rospy.init_node("keyboard")
        controller = TeleopController()
        while not rospy.is_shutdown():
            key = anykey()
            controller.spin(key)
    except ValueError as e:
        rospy.logerr("value error: %s" % e)

    except rospy.ROSInterruptException as s:
        rospy.logerr("ros error: %s" % s)


if __name__ == "__main__":
    main()
