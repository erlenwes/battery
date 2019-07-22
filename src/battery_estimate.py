#! /usr/bin/env python
import rospy
import math
import numpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32



#Hardcoding values needed for calculations
#Current, Voltage and power calculations of all components in the Robot
#Changed based in rated values for the different components
#Dynamixel is not accurate here as it does not account for torque over normal values
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

class mcu:

    voltage = 1.7
    current = 100
    power = voltage*current*0.001

class raspi:
    voltage_b = 0
    current_b = 0 #mA
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
        current[dyna_counter] = (i*300/0.26)+300
        dyna_counter += 1

    #Idle power_dynamixcel
    power_idle = voltage*current_idle*0.001
    #Power calculation relative to speed
    for i in range(len(current)):
        power_right[i] = voltage*current[i]*0.001
        power_left[i] = voltage*current[i]*0.001

class battery:
    voltage = 11.1 #V
    current_capacity = 1800 #mAh
    energy_max = voltage*current_capacity*0.001*3600 #Ws

class speed:
    #Motor on right side
    linear_right = 0
    #Motor on left side
    linear_left = 0
    #angular is a result of different linear speed in left and right motor
    angular = 0






def power_comp_idle():
     return (+mcu.power+(camera.on*camera.power)+raspi.power_b+raspi.power_bp+sensor.power+dynamixel.power_idle*2)

def power_comp():
    power_number_right = int(speed.linear_right*100)
    power_number_left = int(speed.linear_left*100)
    return mcu.power+(camera.on*camera.power)+raspi.power_b+raspi.power_bp+sensor.power+dynamixel.power_right[power_number_right]+dynamixel.power_left[power_number_left]+dynamixel.power_idle*2

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

def battery_life(currentCharge):
    value = currentCharge/(3600*power_comp())
    lifetime = [0]*2
    lifetime[0] = int(value)
    lifetime[1] = int((value%1)*60)

    return lifetime

def battery_life_idle(currentCharge):
    value = currentCharge/(3600*power_comp_idle())
    lifetime = [0]*2
    lifetime[0] = int(value)
    lifetime[1] = int((value%1)*60)

    return lifetime

#Fucntion to calculate battery level, input is the current charge level of the battery in Ws
def batteryCalc(currentCharge):
    while not rospy.is_shutdown():

        while (speed.linear_right or speed.linear_left) == 0 and not rospy.is_shutdown():

            currentCharge = (new_idle_currentCharge(currentCharge))
            pub.publish(currentCharge)

            print("------------------------------------------")
            print("Turtlebot3 idle")
            print("Power consumption: {}W".format(power_comp_idle()))
            print("Battery: {}%".format(round(100*currentCharge/battery.energy_max,2)))
            print("Linear Speed right: {}m/s".format(speed.linear_right))
            print("Linear Speed left: {}m/s".format(speed.linear_left))
            print("Power to motor: {}W".format(dynamixel.power_idle))
            print("Estimated battery time remaining: {}h {}m".format(battery_life_idle(currentCharge)[0], battery_life_idle(currentCharge)[1]))

            rate.sleep()

        while (speed.linear_right or speed.linear_left) > 0 and not rospy.is_shutdown():
            power_number_right = int(speed.linear_right*100)
            power_number_left = int(speed.linear_left*100)

            currentCharge = (new_currentCharge(currentCharge))
            pub.publish(currentCharge)

            print("------------------------------------------")
            print("Turtlebot3 moving")
            print("Power consumption: {}W".format(power_comp()))
            print("Battery: {}%".format(round(100*currentCharge/battery.energy_max,2)))
            print("Linear Speed right: {}m/s".format(speed.linear_right))
            print("Linear Speed left:{}m/s".format(speed.linear_left))
            print("Power to motor: {}W".format(power_to_motor()))
            print("Estimated battery time remaining: {}h {}m").format(battery_life(currentCharge)[0],battery_life(currentCharge)[1])

            rate.sleep()

def callback_speed_calc(velocity):

    speed.linear_right = abs(velocity.linear.x)
    speed.angular = abs(velocity.angular.z)
    speed.linear_left = speed.linear_right + ang_to_lin(speed.angular)

    if speed.angular > 0  and speed.linear_right == 0:
        speed.linear_right = ang_to_lin(speed.angular)
        speed.linear_left = speed.linear_right

    if speed.linear_left > 0.26:

        rest_speed = speed.linear_left - 0.26
        speed.linear_left = 0.26
        speed.linear_right  = abs(speed.linear_right - rest_speed)





#Running the progam
if __name__ == '__main__':

    rospy.init_node("battery_estimate_node")

    #Rate counter set to 1hz/Rate of power drawn
    rate = rospy.Rate(1)
    sub = rospy.Subscriber('/cmd_vel', Twist, callback_speed_calc)
    pub = rospy.Publisher('/battery_charge', Float32, queue_size=1)

    batteryCalc(battery.energy_max)









