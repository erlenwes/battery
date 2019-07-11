#! /usr/bin/env python
import rospy
import math
import numpy
from geometry_msgs.msg import Twist


rospy.init_node("predicter")


def power_comp_idle():
     return ((camera.on*camera.power)+raspi.power_b+raspi.power_bp+sensor.power+dynamixel.power_idle*2)

def power_comp():
    power_number_right = int(speed.linear_right*100)
    power_number_left = int(speed.linear_left*100)
    return (camera.on*camera.power)+raspi.power_b+raspi.power_bp+sensor.power+dynamixel.power_right[power_number_right]+dynamixel.power_left[power_number_left]+dynamixel.power_idle*2

def new_idle_currentCharge(currentCharge):
    currentCharge -= power_comp_idle()
    return currentCharge

def new_currentCharge(currentCharge):
    currentCharge -= power_comp()
    return currentCharge

def power_to_motor():
    power_number_right = int(speed.linear_right*100)
    power_number_left = int(speed.linear_left*100)
    power = dynamixel.power_right[power_number_right] + dynamixel.power_left[power_number_left]
    return power

def ang_to_lin(angular):
    lin = angular * 0.1435
    return round(lin,2)
#Current, Voltage and power levels
class sensor:
    voltage = 5
    current = 400 #mA
    power = voltage*current*0.001
    #need something to indecate camera on/off, subscribe to a topic publishing this info
class camera:

    voltage = 0
    current = 0
    power = voltage * current*0.001
    on = False


class raspi:
    voltage_b = 5
    current_b = 480 #mA
    power_b = voltage_b*current_b*0.001 #W

    voltage_bp = 5
    current_bp = 950 #mA
    power_bp = voltage_bp*current_bp*0.001 #W

class dynamixel:
    #Power consumption for linear movment speed from 0.1 - 0.26 m/s
    voltage = 11.1
    #Idle current consumption dynamixel
    current_idle = 40 #mA
    current = [0]*27 #mA

    power_right = [0]*27
    power_left = [0]*27

    #Current calculations relative to speed
    dyna_counter = 0
    for i in numpy.arange(0, 0.27, 0.01):
        current[dyna_counter] = (i*300/0.26)
        dyna_counter += 1

    #Idle power_dynamixcel
    power_idle = voltage*current_idle*0.001
    #Power calculation relative to speed
    for i in range(len(current)):
        power_right[i] = voltage*current[i]*0.001
        power_left[i] = voltage*current[i]*0.001

#Battery Data
class battery:
    voltage = 11.1 #V
    current_capacity = 1800 #mAh
    energy_max = voltage*current_capacity*0.001*3600 #Ws

class speed:

    linear_right = 0
    linear_left = 0
    angular = 0


#For keeping track
#Rate counter set to 1hz/Rate of power drawn
rate = rospy.Rate(1)



#Fucntion to calculate battery level, input is the current charge level of the battery in Ws
def batteryCalc(currentCharge):
    while not rospy.is_shutdown():

        while (speed.linear_right or speed.linear_left) == 0 and not rospy.is_shutdown():

            currentCharge = (new_idle_currentCharge(currentCharge))

            print("------------------------------------------")
            print("Robot idle")
            print("Power consumption: " +str(power_comp_idle() )+"W")
            print("Battery: " +str(round(100*currentCharge/battery.energy_max,2))+"%")
            print("Linear Speed right: " +str(speed.linear_right)+"m/s")
            print("Linear Speed left: " +str(speed.linear_left)+"m/s")
            print("Power to motor: "+str(dynamixel.power_idle)+"W")

            rate.sleep()

        while (speed.linear_right or speed.linear_left) > 0 and not rospy.is_shutdown():
            power_number_right = int(speed.linear_right*100)
            power_number_left = int(speed.linear_left*100)

            currentCharge = (new_currentCharge(currentCharge))

            print("------------------------------------------")
            print("Robot moving")
            print("Power consumption: " +str(power_comp())+"J")
            print("Battery: " +str(round(100*currentCharge/battery.energy_max,2))+"%")
            print("Linear Speed right: " +str(speed.linear_right)+"m/s")
            print("Linear Speed left: " +str(speed.linear_left)+"m/s")
            print("Power to motor: "+str(power_to_motor())+"W")

            rate.sleep()



def callback(msg):

    speed.linear_right = abs(msg.linear.x)
    speed.angular = abs(msg.angular.z)
    speed.linear_left = speed.linear_right + ang_to_lin(speed.angular)

    if speed.angular > 0  and speed.linear_right == 0:
        speed.linear_right = ang_to_lin(speed.angular)
        speed.linear_left = speed.linear_right

    if speed.linear_left > 0.26:

        rest_speed = speed.linear_left - 0.26
        speed.linear_left = 0.26
        speed.linear_right  = abs(speed.linear_right - rest_speed)


#Running the progam
sub = rospy.Subscriber('/cmd_vel', Twist, callback)
batteryCalc(battery.energy_max)
#Kepping subscription while node running
rospy.spin()









