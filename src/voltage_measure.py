#! /usr/bin/env python
from __future__ import division
import rospy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32





def callback_voltage(msg):

    f=open("/home/user/catkin_ws/src/batterymodule/example.txt","a+")

    f.write("{},".format(msg.voltage))

    f.close()

    print("Voltage: ".format(msg.voltage))

    rate.sleep()

if __name__ == '__main__':


    rospy.init_node('voltage_measure_node')

    rate = rospy.Rate(1)

    sub_main = rospy.Subscriber('/battery_state', Float32, callback_voltage)

    rospy.spin()

















