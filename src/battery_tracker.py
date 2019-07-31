#! /usr/bin/env python
import rospy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32

rospy.init_node('battery_tracker')

voltage = [12.6, 12.45, 12.33, 12.25, 12.07, 11.95, 11.86, 11.74, 11.62, 11.56, 11.51, 11.45, 11.39, 11.36, 11.30, 11.24, 11.18, 11.12, 11.06, 10.83, 9.82] #V

counter = 0

inc_const = 0.004

voltage_full = [0]*(len(voltage)-1)*int(1/inc_const)

rate = rospy.Rate(1)

for i in range(len(voltage)-1):

        inc = 0.0

        for y in range(int(1/inc_const)):

            voltage_full[counter] = voltage[i]+(inc*(voltage[i+1]-voltage[i])/((y+1)-y))

            counter += 1

            inc += 0.004

def callback_voltage(msg):

    percent = 100


    for i in range(len(voltage_full)):

        if msg.voltage >= voltage_full[i-1]:

            percent = 1-(i*len(voltage_full)*100)

    pub.publish(percent)
    if percent < 15:
        print("Battery below 15%, please recharge battery")

    elif percent < 6:
        print("Battery in critical condition, recharge now!")

    else:
        print("Battery: {} %".format(percent))

    print("Reading from topic {}".format(msg.percent))

    rate.sleep()


def listener():

    sub = rospy.Subscriber('/battery_state', BatteryState, callback_voltage)

    pub = rospy.Publisher('/battery_charge', Float32, queue_size=1)

    rospy.spin()

listener()