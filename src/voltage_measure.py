#! /usr/bin/env python
from __future__ import division
import rospy
from sensor_msgs.msg import BatteryState

def callback_voltage(msg):

    global voltage
    voltage = msg.voltage

def voltage_slow():

    f=open("/home/sunniva/catkin_ws/src/voltage.txt","a+")

    f.write("{},".format(voltage))

    f.close()

    print("Voltage: {} ".format(voltage))

    rate.sleep()

if __name__ == '__main__':

    voltage = 0

    rospy.init_node('voltage_measure_node')

    rate = rospy.Rate(1)

    sub_main = rospy.Subscriber('/battery_state', BatteryState, callback_voltage)

    while not rospy.is_shutdown():

        voltage_slow()



















