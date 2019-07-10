#! /usr/bin/env python
import rospy
from battery_state.msg import BatteryState

rospy.init_node('batteryTracker')
rate = rospy.Rate(0.1)


def callback(msg):

    print("Current batterylevel:" +str(msg.percentage))
    rate.sleep()

def listener():

    sub = rospy.Subscriber('sensor_msgs/BatteryState', BatteryState, callback)
    ropsy.spin()

listener()