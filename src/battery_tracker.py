#! /usr/bin/env python
from __future__ import division
import rospy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
import weakref
import numpy

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

def fill_voltage(voltage,voltage_full,inc_const):

    counter = 0

    for i in range(len(voltage)-1):

            inc = 0.0

            for y in range(int(1/inc_const)):

                voltage_full[counter] = voltage[i]+(inc*(voltage[i+1]-voltage[i])/((y+1)-y))

                counter += 1

                inc += inc_const

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



def update_voltage(msg):

    global robot_voltage

    robot_voltage = msg.voltage


def voltage_track():

    percent = 0
    voltage_tracker = 0

    for i in range(len(voltage_full)-1):

        if robot_voltage >= voltage_full[i]:

            percent = (1-(i/len(voltage_full)))*100
            voltage_tracker = i
            break
    for y in range(len(true_voltage_full)-1):

        if robot_voltage >= true_voltage_full[y]:

            true_percent = (1-(y/len(voltage_full)))*100
            break

    time = time_estimation(robot_voltage, voltage_tracker, percent, true_percent)

    pub_main.publish(percent)

    print("Battery voltage:{} ".format(robot_voltage))

    print("Battery: {} %".format(true_percent))

    print("Estimated battery life: {} h {} m".format(time[0], time[1]))

    if percent < 20:

        print(" Low battery, heading back to charging station")

        pub_battery.publish(battery_station)

    rate.sleep()
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


def energy_usage(robot_voltage):

    power_number_right = int(dynamixel.speed_right*100)

    power_number_left = int(dynamixel.speed_left*100)

    static_power = 0

    for obj in static_power_drain.getinstances():

        static_power += obj.power*obj.on

    power = (static_power+dynamixel.power_right[power_number_right]+dynamixel.power_left[power_number_left])*1.1

    return power

def time_estimation(robot_voltage, voltage_tracker, percent, true_percent):

    global real_array_counter
    global real_array
    power = energy_usage(robot_voltage)
    real_percentage_use = 0
    true_lifetime = [0]*2

    true_time = 0
    value = 0

    rest_power = 0


    if real_array_counter % 180 == 0 and real_array_counter <= 180:
        real_array.append(true_percent)

    elif real_array_counter % 180 == 0:

        real_array.append(true_percent)
        real_array.pop()
        real_percentage_use = real_array[0]-real_array[1]
        real_percentage_use = real_percentage_use/60

    if real_percentage_use > 0:
        true_time = true_percent/real_percentage_use
        true_lifetime[0] = int(true_time/3600)
        true_lifetime[1] = int(((true_time/3600)%1)*60)
        print("Measued battery life: {} h {} m".format(true_lifetime[0], true_lifetime[1]))



    for i in range(voltage_tracker, len(voltage_full)-1):

        rest_power += (voltage_full[i]*1.8*36*percent)/(len(voltage_full)-1-voltage_tracker)

    value = rest_power/power

    lifetime = [0]*2

    lifetime[0] = int(value/3600)

    lifetime[1] = int(((value/3600)%1)*60)

    real_array_counter += 1

    return lifetime

if __name__ == '__main__':

    rospy.init_node('battery_tracker')

    lidar = static_power_drain("lidar", 5, 400, True)
    camera = static_power_drain("camera", 5, 250, True)
    mcu = static_power_drain("mcu", 1.7, 100, True)
    raspi = static_power_drain("raspi", 5, 950, True)
    dynamixcel_vampire = static_power_drain("dynamixcel_vamp", 11, 80, True) #40mA per dynamixcel

    dyna_info = fill_dynamixel()
    dynamixel = dynamic_power_drain("dynamixel", dyna_info[0], dyna_info[1], True)

    voltage = [12.6, 12.45, 12.33, 12.25, 12.07, 11.95, 11.86, 11.74, 11.62, 11.56, 11.51, 11.45, 11.39, 11.36, 11.30, 11.24, 11.18, 11.12, 11.06, 10.83, 9.82] #V
    true_voltage = [12.25, 12.07, 11.95, 11.86, 11.82, 11.74, 11.62, 11.56, 11.53, 11.51, 11.45, 11.43, 11.39, 11.36, 11.30, 11.24, 11.18, 11.12, 11.06, 10.83, 9.82] #V

    step_length = 0.002
    voltage_full = [0]*(len(voltage)-1)*int(1/step_length)
    voltage_full = fill_voltage(voltage,voltage_full, step_length)
    true_voltage_full = [0]*(len(true_voltage)-1)*int(1/step_length)
    true_voltage_full = fill_voltage(true_voltage,true_voltage_full, step_length)

    real_array = []
    real_array_counter = 0
    power = 0
    robot_voltage = 12.2

    battery_station = PoseStamped()
    battery_station = fill_battery_station(battery_station)

    sub_main = rospy.Subscriber('/battery_state', BatteryState, update_voltage)
    pub_main = rospy.Publisher('/battery_charge', Float32, queue_size=1)
    pub_battery = rospy.Publisher('/battery_station',PoseStamped, queue_size=1 )

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():

        voltage_track()
