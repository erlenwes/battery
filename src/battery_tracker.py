#! /usr/bin/env python
import rospy
from sensor_msgs.msg import BatteryState
battery_level = 0
rate = rospy.Rate(0.1)


def callback(msg):
    battery_level = round((msg.percentage-1)*1000, 2)
    print("Battery remaining:" +str(battery_level) +"%")
    if battery_level < 20:
        print()
    rate.sleep()

def listener():

    rospy.init_node('battery_tracker')
    sub = rospy.Subscriber('/battery_state', BatteryState, callback)
    ropsy.spin()

listener()