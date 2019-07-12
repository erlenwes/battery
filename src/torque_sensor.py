#! /usr/bin/env python
import rospy
from sensor_msgs.msg import JointState



def callback(msg):

    for i in range(len(msg.name)):
        print("Name: " +msg.name[i])
        print("Position: " +str(msg.position[i])+"m")
        print("Velocity: " +str(msg.velocity[i])+"m/s")
        print("Effort: " +str(msg.effort[i])+"Nm")


def listener():

    rospy.init_node('torque_tracker')
    sub = rospy.Subscriber('/joint_states', JointState, callback)
    ropsy.spin()

listener()