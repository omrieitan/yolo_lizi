#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from yolo_lizi.msg import lizi_imu

pitch = 0.0
roll = 0.0

def imu_cb(data):
    global pitch
    global roll

    pitch = data.pitch
    roll = data.roll

def glove_node():
    global pitch
    global roll

    rospy.init_node('glove_node', anonymous=True)

    rospy.Subscriber("/imu_glove", lizi_imu, imu_cb)
    vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)

    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    while not rospy.is_shutdown():
        if pitch > 0.1 or pitch < -0.1:
            vel_msg.linear.x = pitch
        else:
            vel_msg.linear.x = 0

        if roll > 0.1 or roll < -0.1:
            vel_msg.angular.z = roll
        else:
            vel_msg.angular.z = 0

        vel_pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        glove_node()
    except rospy.ROSInterruptException: pass
