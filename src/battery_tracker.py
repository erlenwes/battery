#! /usr/bin/env python
import rospy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32


rospy.init_node('battery_tracker')
max_battery = 1.095
min_battery = 0.9900
diff = max_battery - min_battery
battery_level = 0
rate = rospy.Rate(0.2)


def callback(msg):
    if msg.percentage > 1:
        battery_level = round((msg.percentage%1)*1000, 2)
        print("Mehtod 1: Battery is at: {} %".format(battery_level))
        pub.publish(battery_level)

    if msg.percentage < 1:
        battery_level = round((msg.percentage - 0.99)*1000,2)
        print("Method 1: Battery is at: {} %".format(battery_level))
        pub.publish(battery_level)

    battery_level = round((msg.percentage-min_battery/diff),2)
    print("Method 2: Battery is at: {} %".format(battery_level))
    #pub.publish(battery_level)

    rate.sleep()


def listener():


    sub = rospy.Subscriber('/battery_state', BatteryState, callback)
    pub = rospy.Publisher('/battery_charge', Float32, queue_size=1)

    rospy.spin()

listener()