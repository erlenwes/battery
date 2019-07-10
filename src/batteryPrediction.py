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
linear_speed = 0
power_number = 0

#Fucntion to calculate battery level, input is the current charge level of the battery in Ws
def batteryCalc(currentCharge):
    while linear_speed == 0 and not rospy.is_shutdown():


        currentCharge = (currentCharge - (power_raspib+power_raspib_plus+power_sensor+idle_power_dynamixcel))
        print("Energy:" +str(round(currentCharge,2))+"Ws")
        print("Battery: " +str(round(100*currentCharge/battery_Energy_max,2))+"%")
        print(idle_power_dynamixcel)
        print linear_speed

        rate.sleep()

    while linear_speed > 0 and not rospy.is_shutdown():
        print("its not idle")
        for i in range(len(power_dynamixcel)):
            if linear_speed == power_dynamixcel[i]:
                power_number = i
            else:
                power_number = 26
        currentCharge = (currentCharge - ((power_raspib+power_raspib_plus+power_sensor)+power_dynamixcel[i]))
        print("Energy:" +str(round(currentCharge,2))+"Ws")
        print("Battery: " +str(round(100*currentCharge/battery_Energy_max,2))+"%")
        print linear_speed)
        print(power_dynamixcel[i])
        rate.sleep()



def callback(msg):
    global linear_speed
    linear_speed = msg.linear.x


sub = rospy.Subscriber('/cmd_vel', Twist, callback)
batteryCalc(battery_Energy_max)
rospy.spin()









