#! /usr/bin/env python
import rospy
import math
import numpy
from geometry_msgs.msg import Twist

rospy.init_node("predicter")
#Current, Voltage and power levels
voltage_sensor = 5
current_sensor = 400 #mA
power_sensor = voltage_sensor*current_sensor*0.001

voltage_raspib = 5
current_raspib = 260 #mA
power_raspib = voltage_raspib*current_raspib*0.001 #W

voltage_raspib_plus = 5
current_raspib_plus = 260 #mA
power_raspib_plus = voltage_raspib_plus*current_raspib_plus*0.001 #W

#Power consumption for linear movment speed from 0.1 - 0.26 m/s
voltage_dynamixel = 11.1
#Idle current consumption dynamixel
idle_current_dynamixel = 40 #mA

current_dynamixcel = [0]*27 #mA
power_dynamixcel = [0]*27

#Current calculations relative to speed
dyna_counter = 0
for i in numpy.arange(0, 0.27, 0.01):
    current_dynamixcel[dyna_counter] = (i*300/0.26)
    dyna_counter += 1

#Idle power_dynamixcel
idle_power_dynamixcel = voltage_dynamixel*idle_current_dynamixel*0.001
#Power calculation relative to speed
for i in range(len(current_dynamixcel)):
    power_dynamixcel[i] = voltage_dynamixel*current_dynamixcel[i]*0.001

#Battery Data

battery_voltage = 11.1 #V
battery_current_capacity = 1800 #mAh
battery_Energy_max = battery_voltage*battery_current_capacity*0.001*3600 #Ws

#Rate counter set to 1hz/Rate of power drawn

rate = rospy.Rate(1)
idle = True
linear_speed = 0
#Fucntion to calculate battery level, input is the current charge level of the battery in Ws
def batteryCalc(currentCharge):
    if linear_speed > 0:
        idle = False
    else:
        idle = True

    while idle and not rospy.is_shutdown():
        rate = rospy.Rate(1)
        print linear_speed
        currentCharge = (currentCharge - (power_raspib+power_raspib_plus+power_sensor+idle_power_dynamixcel))
        print("Energy:" +str(round(currentCharge,2))+"Ws")
        print("Battery: " +str(round(100*currentCharge/battery_Energy_max,2))+"%")
        rate.sleep()
    while not idle and not rospy.is_shutdown():
        rate = rospy.Rate(4)
        print linear_speed
        power_number = 26
        for i in range(len(power_dynamixcel)):
            if linear_speed == power_dynamixcel[i]:
                power_number = i

        currentCharge = (currentCharge - ((power_raspib+power_raspib_plus+power_sensor)*0.25+power_dynamixcel[power_number])
        print("Energy:" +str(round(currentCharge,2))+"Ws")
        print("Battery: " +str(round(100*currentCharge/battery_Energy_max,2))+power_dynamixcel[]"%")
        rate.sleep()



def callback(msg):
    global linear_speed
    linear_speed = msg.linear.x
    print linear_speed





sub = rospy.Subscriber('/cmd_vel', Twist, callback)   # Create a Subscriber object that will listen to the /counter


batteryCalc(battery_Energy_max)
rospy.spin()







