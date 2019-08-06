#! /usr/bin/env python
from __future__ import division
import rospy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped



def fill_voltage(voltage_full):

    counter = 0

    inc_const = 0.004

    for i in range(len(voltage)-1):

            inc = 0.0

            for y in range(int(1/inc_const)):

                voltage_full[counter] = voltage[i]+(inc*(voltage[i+1]-voltage[i])/((y+1)-y))

                counter += 1

                inc += 0.004

    return voltage_full

def fill_battery_station(battery_station):

    battery_station.header.seq = 0
    battery_station.header.stamp = rospy.Time.now()
    battery_station.header.frame_id = "map"
    battery_station.pose.position.x = 0
    battery_station.pose.position.y = 0
    battery_station.pose.position.z = 0.0
    battery_station.pose.orientation.x = 0.0
    battery_station.pose.orientation.y = 0.0
    battery_station.pose.orientation.z = 0
    battery_station.pose.orientation.w = 1.0

    return battery_station


def callback_voltage(msg):

    percent = 0

    for i in range(len(voltage_full)):

        if msg.voltage >= voltage_full[i-1]:

            percent = (1-(i/len(voltage_full)))*100
            break

    pub.publish(percent)

    print("Battery: {} %".format(percent))

    if percent < 15:

        print("Battery below 15%, please recharge battery")

        pub_battery.publish(battery_station)

    rate.sleep()


if __name__ == '__main__':

    rospy.init_node('battery_tracker')

    voltage = [12.6, 12.45, 12.33, 12.25, 12.07, 11.95, 11.86, 11.74, 11.62, 11.56, 11.51, 11.45, 11.39, 11.36, 11.30, 11.24, 11.18, 11.12, 11.06, 10.83, 9.82] #V

    inc_const = 0.004

    voltage_full = [0]*(len(voltage)-1)*int(1/inc_const)

    voltage_full = fill_voltage(voltage_full)

    battery_station = PoseStamped()

    battery_station = fill_battery_station(battery_station)

    rate = rospy.Rate(1)

    sub = rospy.Subscriber('/battery_state', BatteryState, callback_voltage)

    pub = rospy.Publisher('/battery_charge', Float32, queue_size=1)

    pub_battery = rospy.Publisher('/battery_station',PoseStamped, queue_size=1 )

    rospy.spin()
