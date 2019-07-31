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
    voltage = 5
    current = 250
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
    voltage = [0]*27
    #Idle current consumption dynamixel
    current_idle = 40 #mA
    current = 400 #mA
    power_right = [0]*27
    power_left = [0]*27
    counter = 0
    volt_inc = 0.5

    for y in numpy.arange(0, 0.27, 0.01):
        if y < 0.22:
            voltage[counter] = 10
            counter += 1
        else:
            voltage[counter] = 10 + volt_inc
            counter += 1
            volt_inc += 0.5
    voltage[0] = 0
    power_idle = voltage[0]*current_idle*0.001



    #Current calculations relative to speed

    #Idle power_dynamixcel
    #Power calculation relative to speed
    for i in range(len(voltage)):
        power_right[i] = voltage[i]*current*0.001
        power_left[i] = voltage[i]*current*0.001


class battery:
    #Voltage levels in 5% increments from 100 - 0%
    voltage = [12.6, 12.45, 12.33, 12.25, 12.07, 11.95, 11.86, 11.74, 11.62, 11.56, 11.51, 11.45, 11.39, 11.36, 11.30, 11.24, 11.18, 11.12, 11.06, 10.83, 9.82] #V
    counter = 0
    inc_const = 0.004
    power_usage = 0
    voltage_full = [0]*(len(voltage)-1)*int(1/inc_const)
    #Linear interpolation between data points from turtlebot
    for i in range(len(voltage)-1):
        inc = 0.0

        for y in range(int(1/inc_const)):
            voltage_full[counter] = voltage[i]+(inc*(voltage[i+1]-voltage[i])/((y+1)-y))
            counter += 1
            inc += 0.004

    design_current_capacity = 1.8*3600 #As

    used_capacity = 0

    #Voltage level decided by capacity-voltage curve relationship
    voltage_decider = 0

    voltage_level = 0

    tracker = []

class speed:

    #Motor on right side

    linear_right = 0

    #Motor on left side

    linear_left = 0

    #angular is a result of different linear speed in left and right motor

    angular = 0

def power_comp():

    power_number_right = int(speed.linear_right*100)

    power_number_left = int(speed.linear_left*100)

    battery.power_usage = (mcu.power+(camera.on*camera.power)+raspi.power_b+raspi.power_bp+sensor.power+dynamixel.power_right[power_number_right]+dynamixel.power_left[power_number_left]+dynamixel.power_idle*2)*1.1

    battery.tracker.append(battery.power_usage)

    if len(battery.tracker) > (60*5):

        battery.tracker.pop()

    battery.voltage_decider = int((battery.used_capacity/battery.design_current_capacity)*(len(battery.voltage)-1)/battery.inc_const)

    battery.voltage_level = battery.voltage_full[battery.voltage_decider]

    current = battery.power_usage/battery.voltage_level

    battery.used_capacity += current

    return current

def new_currentCharge(currentCharge):
    currentCharge -= power_comp()

    return currentCharge

def power_to_motor():

    power_number_right = int(speed.linear_right*100)

    power_number_left = int(speed.linear_left*100)

    power = dynamixel.power_right[power_number_right] + dynamixel.power_left[power_number_left]+dynamixel.power_idle*2

    return power

def ang_to_lin(angular):

    lin = angular * 0.1435

    return round(lin,2)


def is_moving():

    power_number_right = int(speed.linear_right*100)

    power_number_left = int(speed.linear_left*100)

    moving = False
    if power_number_right > 0 or power_number_left > 0:
        moving  = True

    return moving


def battery_life(currentCharge):

    rest_power = 0

    value = 0

    for i in range(battery.voltage_decider, len(battery.voltage_full)-1):

        rest_power += battery.voltage_full[i]*currentCharge/(len(battery.voltage_full)-1-battery.voltage_decider)

    if len(battery.tracker) == 1:

        value = battery.tracker[0]

    for x in range(len(battery.tracker)-1):

        value += battery.tracker[x]/(len(battery.tracker)-1)


    value = rest_power/(3600*(value+battery.power_usage)/2)

    lifetime = [0]*2

    lifetime[0] = int(value)

    lifetime[1] = int((value%1)*60)

    return lifetime

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

#Fucntion to calculate battery level, input is the current charge level of the battery in Ampere seconds
def batteryCalc(current_capacity):

    while not rospy.is_shutdown():

        state ="idle"
        timer = [0]*2

        if is_moving():
            state = "moving"


        current_capacity = new_currentCharge(current_capacity)

        pub_charge.publish(100*current_capacity/battery.design_current_capacity)

        timer = battery_life(current_capacity)

        print("------------------------------------------")
        print("Turtlebot3 {}".format(state))
        #print("Current capcity left: {}".format(current_capacity))
        print("Battery voltage: {}V".format(battery.voltage_level))
        #print("Used capacity: {}".format(battery.used_capacity))
        #print("voltage decider:{} ".format(battery.voltage_decider))
        print("Battery: {}%".format(round(100*current_capacity/battery.design_current_capacity,2)))
        print("Linear Speed right: {}m/s".format(speed.linear_right))
        print("Linear Speed left: {}m/s".format(speed.linear_left))
        #print("Power to motor: {}W".format(power_to_motor()))
        print("Estimated battery time remaining: {}h {}m").format(timer[0],timer[1])

        rate.sleep()

#Running the progam
if __name__ == '__main__':

    rospy.init_node("batterysim_node")

    #Rate counter set to 1hz/Rate of power drawn
    rate = rospy.Rate(1)
    sub = rospy.Subscriber('/cmd_vel', Twist, callback_speed_calc)
    pub_charge = rospy.Publisher('/battery_charge', Float32, queue_size=1)
    start_cap = 1.8*3600
    battery.used_capacity = battery.design_current_capacity - start_cap
    batteryCalc(start_cap)









