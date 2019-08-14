#! /usr/bin/env python
from __future__ import division
import rospy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import weakref
import numpy

#Making a class for the static power drains in the robot
class static_power_drain():

    _instances = set()

    def __init__(self, name, voltage, current, status):
        self.name = name #name on the component
        self.voltage = voltage #V
        self.current = current #mA
        self.power = voltage*current*0.001 #W
        self.on = status #True/false
        self._instances.add(weakref.ref(self)) #Ref to itself
#Weakref to itself so its easier to add new drais to power calcualtions
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
#Power drains with dynamic levels, for turtlebot only dynamixel wrp rpm and torque
class dynamic_power_drain():

    _instances = set()

    def __init__(self, name, voltage, current, on):
        self.name = name #Name of the component
        self.voltage = voltage #V
        self.current = current #mA
        self.power_right = [0]*len(voltage) #empty array based voltage
        self.power_left = [0]*len(voltage) #empty array based voltage

	#Power based on speed from 0-0.26 m/s, as voltage varies
        for i in range(len(voltage)):
	#fillinf the array for both motors
            self.power_right[i] = (voltage[i]*current[i]*0.001)
            self.power_left[i] = (voltage[i]*current[i]*0.001)

        self.on = on #True/false
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
    #variables to keep track of speed, to omit global variables
    speed_left = 0
    speed_right = 0
    speed_angular = 0

#Filling the array for the dynamixcel
def fill_dynamixel():

    retur = [0]*2
    #Power consumption for linear movment speed from 0.1 - 0.26 m/s
    voltage = [0]*27

    current = [400]*27 #mA

    counter = 0

    volt_inc = 0.5
    #array filled with 
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
#interpolating between each number in the array given, step length based on input
    counter = 0

    for i in range(len(voltage)-1):

            inc = 0.0

            for y in range(int(1/inc_const)):

                voltage_full[counter] = voltage[i]+(inc*(voltage[i+1]-voltage[i])/((y+1)-y))

                counter += 1

                inc += inc_const

    return voltage_full

def fill_battery_station(battery_station):
#Setting the location of the battery station in a PostStamped data type
    battery_station.header.seq = 0
    battery_station.header.stamp = rospy.Time.now()
    battery_station.header.frame_id = "map" #name of the map
    battery_station.pose.position.x = 0
    battery_station.pose.position.y = 0
    battery_station.pose.position.z = 0.0
    battery_station.pose.orientation.x = 0.0 #x coord in the costmap
    battery_station.pose.orientation.y = 0.0 #y coord in the costmap
    battery_station.pose.orientation.z = 0
    battery_station.pose.orientation.w = 1.0

    return battery_station



def update_voltage(msg):
#Updating the actual voltage level read from the robot
    global robot_voltage

    robot_voltage = msg.voltage

#The main fucntion of the program
def voltage_track():

    percent = 0
    voltage_tracker = 0 #index in the voltage arary for keeping track of voltage outside of func
    #Comparing the voltage of the robot to the voltage level in the battery array, breaking the loop when the voltage mathces, then finding the percentage based on the SOC vs votlage curve
    for i in range(len(voltage_full)-1):

        if robot_voltage >= voltage_full[i]:

            percent = (1-(i/len(voltage_full)))*100
            voltage_tracker = i

            break
    #Comparing the voltage of the battery for display purposes. As current increases the SOC will change, for this lipo battery the max charge is 12.2 voltage, but the capacity will be eq to a 80% 12.6 battery.
    for y in range(len(true_voltage_full)-1):

        if robot_voltage >= true_voltage_full[y]:

            true_percent = (1-(y/len(voltage_full)))*100

            break
    #Estimating the remaining battery time for the robot
    time = time_estimation(robot_voltage, voltage_tracker, percent, true_percent)
    #publishing the battery percentage to a topic, for use in path_predictor
    pub_main.publish(true_percent)
    
    #Printing the values
    print("-------------------------------")

    print("Battery voltage: {}V".format(round(robot_voltage,3)))

    print("Battery percentage: {}%".format(true_percent))

    print("Speed motor right: {}m/s".format(dynamixel.speed_right))

    print("Speed motor left: {}m/s".format(dynamixel.speed_left))

    print("Estimated battery life: {}h {}m".format(time[0], time[1]))
    
    #Sending the robot back to the station when battery below a threshold
    if true_percent < 20:

        print(" Low battery, heading back to charging station")

        pub_battery.publish(battery_station)

    rate.sleep()

#Calculating speed of each motor based on linear and angular speed
def callback_speed_calc(velocity):

    dynamixel.speed_right = abs(velocity.linear.x) #m/s

    dynamixel.speed_angular = abs(velocity.angular.z)#rad/s
    #Assuming the same motor always adjusing for the turning
    dynamixel.speed_left = dynamixel.speed_right + ang_to_lin(dynamixel.speed_angular)#m/s  
    
    #when rotating on the spot, right and left motor have the same speed
    if dynamixel.speed_angular > 0  and dynamixel.speed_right == 0:

        dynamixel.speed_right = ang_to_lin(dynamixel.speed_angular)

        dynamixel.speed_left = dynamixel.speed_right
    
    #when turning with linear speed, increase one motor speed equal to the inc in ang speed
    #if max speed decrease the other speed or reverse
    if dynamixel.speed_left > 0.26:

        rest_speed = dynamixel.speed_left - 0.26

        dynamixel.speed_left = 0.26

        dynamixel.speed_right  = abs(dynamixel.speed_right - rest_speed)

#calculating the energy used by the robot for use in time estimation
def energy_usage(robot_voltage):

    power_number_right = int(dynamixel.speed_right*100) #transforming the speed of the robot to an index in the voltage-speed array

    power_number_left = int(dynamixel.speed_left*100)
    
    static_power = 0 #defining the variable

    #adding the power from all the static drains, chekcing if active or not
    for obj in static_power_drain.getinstances():

        static_power += obj.power*obj.on

    power = (static_power+dynamixel.power_right[power_number_right]+dynamixel.power_left[power_number_left])*1.1

    return power

#trasforming angular speed(rad/s) to linear speed (m/s)
def ang_to_lin(angular):

    lin = angular * 0.1435

    return round(lin,2)


def time_estimation(robot_voltage, voltage_tracker, percent, true_percent):

    power = energy_usage(robot_voltage)
    real_percentage_use = 0
    true_lifetime = [0]*2

    true_time = 0
    value = 0

    rest_power = 0


    for i in range(voltage_tracker, len(voltage_full)-1):

        rest_power += (voltage_full[i]*1.8*36*percent)/(len(voltage_full)-1-voltage_tracker)

    value = rest_power/power

    lifetime = [0]*2

    lifetime[0] = int(value/3600)

    lifetime[1] = int(((value/3600)%1)*60)

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
    true_voltage = [12.25, 11.98, 11.95, 11.86, 11.82, 11.74, 11.62, 11.56, 11.53, 11.51, 11.45, 11.43, 11.39, 11.36, 11.30, 11.24, 11.18, 11.12, 11.06, 10.83, 9] #V

    step_length = 0.002
    voltage_full = [0]*(len(voltage)-1)*int(1/step_length)
    voltage_full = fill_voltage(voltage,voltage_full, step_length)
    true_voltage_full = [0]*(len(true_voltage)-1)*int(1/step_length)
    true_voltage_full = fill_voltage(true_voltage,true_voltage_full, step_length)

    power = 0
    robot_voltage = 12.2

    battery_station = PoseStamped()
    battery_station = fill_battery_station(battery_station)

    sub_main = rospy.Subscriber('/battery_state', BatteryState, update_voltage)
    pub_main = rospy.Publisher('/battery_charge', Float32, queue_size=1)
    pub_battery = rospy.Publisher('/battery_station',PoseStamped, queue_size=1 )
    sub = rospy.Subscriber('/cmd_vel', Twist, callback_speed_calc)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():

        voltage_track()
