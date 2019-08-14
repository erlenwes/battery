#! /usr/bin/env python
import rospy
import numpy
from math import sqrt
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from std_msgs.msg import Float32, String
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from nav_msgs.srv import GetPlan, GetPlanRequest
import weakref


class battery_sim():

    def __init__(self, type,cells,voltage_curve):
        self.type = type
        self.cells = cells
        self.voltage_full = voltage_curve
	self.design_current_capacity = 1.8*3600
	self.used_capacity = 0
	self.power_usage = 0
	self.voltage_decider = 0
	self.voltage_level = 0
	self.rem_capacity = 0

    
    #Voltage level decided by capacity-voltage curve relationship


class robot_params():

    def __init__(self,name,maximum, power_max, current_capacity, before):
        self.name = name
        self.maximum = maximum
        self.power_max = power_max
        self.current_capacity = current_capacity
        self.before = before
        self.max_linvel_x = (rospy.get_param('/move_base/DWAPlannerROS/max_vel_x')) #m/s
        self.max_linacc_x = (rospy.get_param('/move_base/DWAPlannerROS/acc_lim_x'))  #m/s^2
        self.max_angvel_z = (rospy.get_param('/move_base/DWAPlannerROS/max_rot_vel')) #rad/s
        self.current = 0
	self.status_before = 0
	self.after = 0
	self.to_goal_initial = 0
	self.to_goal = 0
	self.back_to_station = 0
	self.counter = 0


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
	self.speed_left = 0
	self.speed_right = 0
	self.speed_angular = 0
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

def distance(a, b):

	del_x = a.pose.position.x - b.pose.position.x

	del_y = a.pose.position.y - b.pose.position.y

	return sqrt(del_x**2 + del_y**2)


def load_charging_station(coord_x, coord_y):

    charging_station = PoseStamped()

    charging_station.header.seq = 0
    charging_station.header.stamp = rospy.Time.now()
    charging_station.header.frame_id = "map"
    #Coordinates of charging staion
    #---------------------------------------------------
    charging_station.pose.position.x = coord_x
    charging_station.pose.position.y = coord_y
    #---------------------------------------------------
    charging_station.pose.position.z = 0.0
    charging_station.pose.orientation.x = 0.0
    charging_station.pose.orientation.y = 0.0
    #Orientation of position in charging back_to_station
    #---------------------------------------------------
    charging_station.pose.orientation.z = 0
    charging_station.pose.orientation.w = 1.0
    #---------------------------------------------------

    return charging_station

#Checking if the robot is on its way to a goal or if the goal is reached. Measuring the actual battery consumption from start to goal reached
def callback_goal_status(goal_status):
    #If prev status is moving and the new status is goal reached, then a new goal is reached and the function prints relative difference in energy measured at start and goal reached.

    if len(goal_status.status_list) >= 1:
        try:

            if ((goal_status.status_list[len(goal_status.status_list)-1].status) == 3 and (robot.status_before == 1)):

                robot.status_before = (goal_status.status_list[len(goal_status.status_list)-1].status)

                if round((robot.before-robot.after),3) > 0 and round((robot.before-robot.after),3) < 100:

                    print("Measured battery usage: {} %".format(round((robot.before-robot.after),3)))

                elif round((robot.before-robot.after),3) == 0:

                    print("Distance to short for accurate battery measurement")

                else:

                    print("Error: Distance too short or changing goal before old goal finished")

            #If current status is goal reached just update charge_before
            if (goal_status.status_list[len(goal_status.status_list)-1].status == 3):

                robot.before = robot.current

                robot.status_before = (goal_status.status_list[len(goal_status.status_list)-1].status)

            #If current status is moving then update charge.after until the new status is goal reached
            if (goal_status.status_list[len(goal_status.status_list)-1].status) == 1:

                robot.after = robot.current

                robot.status_before = (goal_status.status_list[len(goal_status.status_list)-1].status)

        except:
            print("Error with the callback goal tracker")

    elif len(goal_status.status_list) == 0:

        robot.before = robot.current



#Getting path from the current goal back to the designated charging dist_to_station
def get_path(start, goal, tol = 1):
    #Setting the counter for travel dist = 0 so that only the initial length is taken, the always updating path is taken care of later
    robot.counter = 0

    charging_station = load_charging_station(0,0)

    rospy.wait_for_service('/move_base/NavfnROS/make_plan')

    connection_service = rospy.ServiceProxy('/move_base/NavfnROS/make_plan', GetPlan)

    param = GetPlanRequest()

    param.start = start

    param.goal = goal

    param.tolerance = tol

    path = connection_service(param)

    robot.back_to_station = path_distance(path.plan, increment_gain = 1)

    eng_path = calc_power_usage(robot.to_goal_initial, robot.back_to_station)

    threshold = 25

    if (robot.current - ((100*(eng_path[0][0]+eng_path[2]))/robot.power_max)) > threshold:

        print("-------------------------------------------------------------")

        print("New goal received")

        print("Initial distance to goal: {} m".format(round(robot.to_goal_initial,3)))

        print("Distance from goal to charging station: {} m".format(round(robot.back_to_station,3)))

        print("Estimated battery consumption for next goal: {} %".format(round(100*eng_path[0][0]/robot.power_max,2)))

        print("Estimated battery consumption required for total operation: {}%".format(round(100*eng_path[2]/robot.power_max,2)))


    else:

        print("-------------------------------------------------------------")

        print("Battery threshold {} %".format(threshold))

        print("Estimated battery usage for next goal and return is {} %, current battery is at {} %".format(((100*(eng_path[0][0]+eng_path[2]))/robot.power_max), robot.current))

        print("Battery too low for this goal, returning to station")

        pub.publish(charging_station)



def path_distance(path, increment_gain = 5):

    index = 0

    length = 0

    max_index = len(path.poses) - 1

    while not rospy.is_shutdown():

		if index + increment_gain < max_index:

		    length += distance(path.poses[index], path.poses[index + increment_gain])

		    index += increment_gain

		else:
                    try:
		        length += distance(path.poses[index], path.poses[max_index])
		        break
            	    except IndexError:
                        print("Goal out of reach")
                        break
    return length

def power_to_goal(time_to_goal):

    power_number_right = int(robot.max_linvel_x*100)

    energy_used = 0

    retur = [0]*2

    power_number_left = power_number_right

    retur[1] = (robot.current*(battery.design_current_capacity))/100

    for i in range(int(time_to_goal)):
        static_power = 0

        voltage_decider = int((battery.used_capacity/battery.design_current_capacity)*len(battery.voltage_full))

        if voltage_decider > len(battery.voltage_full)-1:

            voltage_decider = len(battery.voltage_full)/2

        for obj in static_power_drain.getinstances():

                static_power += obj.power*obj.on

        power = (static_power+dynamixel.power_right[power_number_right]+dynamixel.power_left[power_number_left])

        voltage = battery.voltage_full[voltage_decider]

        current = power/voltage

        retur[0] += current*voltage

        retur[1] -= current

    return retur

def power_to_dock(time_to_dock, currentCharge):

    power_number_right = int(robot.max_linvel_x*100)

    energy_used = 0

    power_number_left = power_number_right

    new_currentCharge = currentCharge

    for i in range(int(time_to_dock)):

        static_power = 0

        voltage_decider = int(((battery.design_current_capacity-new_currentCharge)/battery.design_current_capacity)*len(battery.voltage_full))

        if voltage_decider > len(battery.voltage_full)-1:

            voltage_decider = len(battery.voltage_full)/2

        for obj in static_power_drain.getinstances():

                static_power += obj.power*obj.on

        power = (static_power+dynamixel.power_right[power_number_right]+dynamixel.power_left[power_number_left])

        voltage = battery.voltage_full[voltage_decider]

        current = power/voltage

        energy_used += current*voltage

        new_currentCharge -= current

    return energy_used


def calc_power_usage(dist_to_goal, dist_to_dock):

    time_to_goal = (dist_to_goal/robot.max_linvel_x)*0.9 + (dist_to_goal/(0.5*robot.max_linvel_x))*0.1

    time_to_dock = (dist_to_dock/robot.max_linvel_x)*0.9 + (dist_to_dock/(0.5*robot.max_linvel_x))*0.1

    power_usage = [0]*3

    power_usage[0] = power_to_goal(time_to_goal)

    power_usage[1] = power_to_dock(time_to_dock,power_usage[0][1] )

    power_usage[2] = power_usage[0][0] + power_usage[1]

    return power_usage


def callback_power_comp(capacity):

    robot.current = capacity.data


def callback_path(path):

    if robot.counter < 1:

        robot.to_goal_initial = path_distance(path, increment_gain = 1)

        robot.counter += 1

    else:

        robot.to_goal = path_distance(path, increment_gain = 1)

        robot.counter += 1


def callback_goal(goal):

    charging_station = load_charging_station(coord_x = 0, coord_y = 0)

    get_path(goal.goal.target_pose, charging_station, tol = 1)


if __name__ == '__main__':

    # Initializing

    rospy.init_node("path_power_predicter_real_node")

    lidar = static_power_drain("lidar", 5, 400, True)
    camera = static_power_drain("camera", 5, 250, False)
    mcu = static_power_drain("mcu", 1.7, 100, True)
    raspi = static_power_drain("raspi", 5, 950, True)
    dynamixcel_vampire = static_power_drain("dynamixcel_vamp", 11, 80, True) #40mA per dynamixcel

    dyna_info = fill_dynamixel()
    dynamixel = dynamic_power_drain("dynamixel", dyna_info[0], dyna_info[1], True)

    battery_voltages = fill_battery()
    battery = battery_sim("lipo",3,battery_voltages)

    robot = robot_params("turtlebot3",1.8*3600,1.8*11.1*3600,1.8*3600,1.8*3600)

    rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, callback_goal)

    rospy.Subscriber("move_base/NavfnROS/plan", Path, callback_path)

    rospy.Subscriber("/battery_charge", Float32, callback_power_comp)

    rospy.Subscriber("/move_base/status", GoalStatusArray, callback_goal_status)

    pub = rospy.Publisher('/status_path', PoseStamped, queue_size=1)


    rospy.spin()
