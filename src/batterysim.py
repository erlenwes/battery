#! /usr/bin/env python
import rospy
import math
import numpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import weakref



#Hardcoding values needed for calculations
#Current, Voltage and power calculations of all components in the Robot
#Changed based in rated values for the different components
#Dynamixel is not accurate here as it does not account for torque over normal values

class static_power_drain():

    _instances = set()

    def __init__(self, name, voltage, current, status):
        self.name = name
        self.voltage = voltage
        self.current = current
        self.power = voltage*current*0.001
        self.on = status
        self._instances.add(weakref.ref(self))

    @classmethod
    def getinstances(cls):
        dead = set()
        for ref in cls._instances:
            obj = ref()
            if obj is not None:
                yield obj
            else:
                dead.add(ref)
        cls._instances -= dead

class dynamic_power_drain():

    _instances = set()

    def __init__(self, name, voltage, current, on):
        self.name = name
        self.voltage = voltage
        self.current = current
        self.power_right = [0]*len(voltage)
        self.power_left = [0]*len(voltage)

        for i in range(len(voltage)):

            self.power_right[i] = (voltage[i]*current[i]*0.001)
            self.power_left[i] = (voltage[i]*current[i]*0.001)

        self.on = on
        self._instances.add(weakref.ref(self))

    @classmethod
    def getinstances(cls):
        dead = set()
        for ref in cls._instances:
            obj = ref()
            if obj is not None:
                yield obj
            else:
                dead.add(ref)
        cls._instances -= dead

    speed_left = 0
    speed_right = 0
    speed_angular = 0


class battery_sim():
    #Voltage levels in 5% increments from 100 - 0%
    def __init__(self, type,cells,voltage_curve):
        self.type = type
        self.cells = cells
        self.voltage_full = voltage_curve

    power_usage = 0
    design_current_capacity = 1.8*3600 #As
    used_capacity = 0
    #Voltage level decided by capacity-voltage curve relationship
    voltage_decider = 0
    voltage_level = 0
    tracker = []

def fill_battery():

    voltage = [12.6, 12.45, 12.33, 12.25, 12.07, 11.95, 11.86, 11.74, 11.62, 11.56, 11.51, 11.45, 11.39, 11.36, 11.30, 11.24, 11.18, 11.12, 11.06, 10.83, 9.82] #V

    inc_const = 0.002

    counter = 0

    voltage_full = [0]*(len(voltage)-1)*int(1/inc_const)

    #Linear interpolation between data points from turtlebot
    for i in range(len(voltage)-1):

        inc = 0

        for y in range(int(1/inc_const)):

            voltage_full[counter] = voltage[i]+(inc*(voltage[i+1]-voltage[i])/((y+1)-y))

            counter += 1

            inc += 0.002

    return voltage_full



def fill_dynamixel():

    retur = [0]*2
    #Power consumption for linear movment speed from 0.1 - 0.26 m/s
    voltage = [0]*27

    current = [400]*27 #mA

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

    retur[0] = voltage
    retur[1] = current
    return retur

def power_comp():

    power_number_right = int(dynamixel.speed_right*100)

    power_number_left = int(dynamixel.speed_left*100)

    static_power = 0

    current = 0

    for obj in static_power_drain.getinstances():

        static_power += obj.power*obj.on
        current += obj.current*obj.on

    battery.power_usage = (static_power+dynamixel.power_right[power_number_right]+dynamixel.power_left[power_number_left])

    battery.tracker.append(battery.power_usage)

    if len(battery.tracker) > (60*5):

        battery.tracker.pop()

    battery.voltage_decider = int((battery.used_capacity/battery.design_current_capacity)*len(battery.voltage_full))

    battery.voltage_level = battery.voltage_full[battery.voltage_decider]

    current = battery.power_usage/battery.voltage_level

    battery.used_capacity += current

    return current

def new_currentCharge(currentCharge):

    currentCharge -= power_comp()

    return currentCharge

def power_to_motor():

    power_number_right = int(dynamixel.speed_right*100)

    power_number_left = int(dynamixel.speed_left*100)

    power = dynamixel.power_right[power_number_right] + dynamixel.power_left[power_number_left]

    return power

def ang_to_lin(angular):

    lin = angular * 0.1435

    return round(lin,2)


def is_moving():

    power_number_right = int(dynamixel.speed_right*100)

    power_number_left = int(dynamixel.speed_left*100)

    moving = False

    if power_number_right > 0 or power_number_left > 0:

        moving  = True

    return moving


def battery_life(currentCharge):

    rest_power = 0

    value = 0

    for i in range(battery.voltage_decider, len(battery.voltage_full)-1):

        rest_power += (battery.voltage_full[i]*currentCharge)/(len(battery.voltage_full)-1-battery.voltage_decider)

    if len(battery.tracker) == 1:

        value = battery.tracker[0]

    for x in range(len(battery.tracker)-1):

        value += battery.tracker[x]/(len(battery.tracker)-1)

    used_power = (value+battery.power_usage)/2

    value = rest_power/used_power

    lifetime = [0]*2

    lifetime[0] = int(value/3600)

    lifetime[1] = int(((value/3600)%1)*60)

    print("Ws", rest_power)

    return lifetime

def callback_speed_calc(velocity):

    dynamixel.speed_right = abs(velocity.linear.x)

    dynamixel.speed_angular = abs(velocity.angular.z)

    dynamixel.speed_left = dynamixel.speed_right + ang_to_lin(dynamixel.speed_angular)

    if dynamixel.speed_angular > 0  and dynamixel.speed_right == 0:

        dynamixel.speed_right = ang_to_lin(dynamixel.speed_angular)

        dynamixel.speed_left = dynamixel.speed_right

    if dynamixel.speed_left > 0.26:

        rest_speed = dynamixel.speed_left - 0.26

        dynamixel.speed_left = 0.26

        dynamixel.speed_right  = abs(dynamixel.speed_right - rest_speed)

#Fucntion to calculate battery level, input is the current charge level of the battery in Ampere seconds
def batteryCalc(current_capacity):

    while not rospy.is_shutdown():

        state ="idle"

        timer = [0]*2

        if is_moving():

            state = "moving"


        current_capacity = new_currentCharge(current_capacity)

        pub_charge.publish(current_capacity)

        timer = battery_life(current_capacity)

        print("------------------------------------------")

        print("Turtlebot3 status: {}".format(state))

        print("Battery voltage: {}V".format(battery.voltage_level))

        print("Battery: {}%".format(round(100*current_capacity/battery.design_current_capacity,2)))

        print("Motor speed right: {}m/s".format(dynamixel.speed_right))

        print("Motor speed left: {}m/s".format(dynamixel.speed_left))

        print("Estimated battery time remaining: {}h {}m").format(timer[0],timer[1])

        rate.sleep()


#Running the progam
if __name__ == '__main__':

    rospy.init_node("batterysim_node")

    #Rate counter set to 1hz/Rate of power drawn
    rate = rospy.Rate(1)

    lidar = static_power_drain("lidar", 5, 400, True)
    camera = static_power_drain("camera", 5, 250, False)
    mcu = static_power_drain("mcu", 1.7, 100, True)
    raspi = static_power_drain("raspi", 5, 950, True)
    dynamixcel_vampire = static_power_drain("dynamixcel_vamp", 11, 80, True) #40mA per dynamixcel

    dyna_info = fill_dynamixel()
    dynamixel = dynamic_power_drain("dynamixel", dyna_info[0], dyna_info[1], True)

    battery_voltages = fill_battery()
    battery = battery_sim("lipo",3,battery_voltages)

    sub = rospy.Subscriber('/cmd_vel', Twist, callback_speed_calc)
    pub_charge = rospy.Publisher('/battery_charge', Float32, queue_size=1)

    start_cap = 1.8*3600
    battery.used_capacity = battery.design_current_capacity - start_cap

    batteryCalc(start_cap)









