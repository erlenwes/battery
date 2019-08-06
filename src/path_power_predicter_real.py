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

    rem_capacity = 0

    voltage_decider = 0

    voltage_level = 0

class travel_dist:

    to_goal_initial = 0

    to_goal = 0

    back_to_station = 0

    counter = 0

class robot_params:

    max_linvel_x = (rospy.get_param('/move_base/DWAPlannerROS/max_vel_x')) #m/s

    max_linacc_x = (rospy.get_param('/move_base/DWAPlannerROS/acc_lim_x'))  #m/s^2

    max_angvel_z = (rospy.get_param('/move_base/DWAPlannerROS/max_rot_vel')) #rad/s


class sensor:

    voltage = 5

    current = 400 #mA

    power = voltage*current*0.001

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


    for i in range(len(voltage)):

        power_right[i] = voltage[i]*current*0.001

        power_left[i] = voltage[i]*current*0.001

class charge:

    current = 100

    maximum = 1.8*11.1*3600

    current_capacity = 1.8*3600

    status_before = 0

    before = 0

    after = 0

def distance(a, b):

	del_x = a.pose.position.x - b.pose.position.x

	del_y = a.pose.position.y - b.pose.position.y

	return sqrt(del_x**2 + del_y**2)


def load_charging_station(coord_x = -2.32499957085, coord_y = 1.19209289551e-07):

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
    charging_station.pose.orientation.z = -1.38615431446e-07
    charging_station.pose.orientation.w = 1.0
    #---------------------------------------------------

    return charging_station

#Checking if the robot is on its way to a goal or if the goal is reached. Measuring the actual battery consumption from start to goal reached
def callback_goal_status(goal_status):
    #If prev status is moving and the new status is goal reached, then a new goal is reached and the function prints relative difference in energy measured at start and goal reached.

    if len(goal_status.status_list) >= 1:
        try:

            if ((goal_status.status_list[len(goal_status.status_list)-1].status) == 3 and (charge.status_before == 1)):

                charge.status_before = (goal_status.status_list[len(goal_status.status_list)-1].status)

                print("Actual battery usage: {} %".format(round((charge.before-charge.after),3)))

            #If current status is goal reached just update charge_before
            if (goal_status.status_list[len(goal_status.status_list)-1].status == 3):

                charge.before = charge.current

                charge.status_before = (goal_status.status_list[len(goal_status.status_list)-1].status)

            #If current status is moving then update charge.after until the new status is goal reached
            if (goal_status.status_list[len(goal_status.status_list)-1].status) == 1:

                charge.after = charge.current

                charge.status_before = (goal_status.status_list[len(goal_status.status_list)-1].status)

        except:
            print("Error with the callback goal tracker")

    elif len(goal_status.status_list) == 0:

        charge.before = charge.current



#Getting path from the current goal back to the designated charging dist_to_station
def get_path(start, goal, tol = 1):
    #Setting the counter for travel dist = 0 so that only the initial length is taken, the always updating path is taken care of later
    travel_dist.counter = 0

    rospy.wait_for_service('/move_base/NavfnROS/make_plan')

    connection_service = rospy.ServiceProxy('/move_base/NavfnROS/make_plan', GetPlan)

    param = GetPlanRequest()

    param.start = start

    param.goal = goal

    param.tolerance = tol

    path = connection_service(param)

    travel_dist.back_to_station = path_distance(path.plan, increment_gain = 1)

    eng_path = calc_power_usage(travel_dist.to_goal_initial, travel_dist.back_to_station)

    threshold = 10

    if (charge.current - ((100*(eng_path[0][0]+eng_path[2]))/charge.maximum)) > threshold:

        print("-------------------------------------------------------------")

        print("New goal received")

        print("Initial distance to goal: {} m".format(round(travel_dist.to_goal_initial,3)))

        print("Distance from goal to charging station: {} m".format(round(travel_dist.back_to_station,3)))

        print("Estimated battery consumption for next goal: {} %".format(round(100*eng_path[0][0]/charge.maximum,2)))

        print("Estimated battery consumption required for total operation: {}%".format(round(100*eng_path[2]/charge.maximum,2)))


    else:

        print("-------------------------------------------------------------")

        print("Battery threshold {} %".format(threshold))

        print("Estimated battery usage for next goal and return is {} %, current battery is at {} %".format(((100*(eng_path[0][0]+eng_path[2]))/charge.maximum), charge.current))

        print("Battery too low for this goal, choose a new goal")

        pub.publish("DONT DO IT")



def path_distance(path, increment_gain = 5):

    index = 0

    length = 0

    max_index = len(path.poses) - 1

    while not rospy.is_shutdown():

		if index + increment_gain < max_index:

		    length += distance(path.poses[index], path.poses[index + increment_gain])

		    index += increment_gain

		else:

			length += distance(path.poses[index], path.poses[max_index])

			break

    return length

def power_to_goal(time_to_goal):

    power_number_right = int(robot_params.max_linvel_x*100)

    energy_used = 0

    retur = [0]*2

    power_number_left = power_number_right

    retur[1] = (charge.current*(battery.design_current_capacity))/100

    for i in range(int(time_to_goal)):

        voltage_decider = int(((battery.design_current_capacity-retur[1])/battery.design_current_capacity)*(len(battery.voltage)-1)/battery.inc_const)

        power = (mcu.power+(camera.on*camera.power)+raspi.power_bp+sensor.power+dynamixel.power_right[power_number_right]+dynamixel.power_left[power_number_left]+dynamixel.power_idle*2)*1.1

        voltage = battery.voltage_full[voltage_decider]

        current = power/voltage

        retur[0] += current*voltage

        retur[1] -= current

    return retur

def power_to_dock(time_to_dock, currentCharge):

    power_number_right = int(robot_params.max_linvel_x*100)

    energy_used = 0

    power_number_left = power_number_right

    new_currentCharge = currentCharge

    for i in range(int(time_to_dock)):

        voltage_decider = int(((battery.design_current_capacity-new_currentCharge)/battery.design_current_capacity)*(len(battery.voltage)-1)/battery.inc_const)

        power = (mcu.power+(camera.on*camera.power)+raspi.power_bp+sensor.power+dynamixel.power_right[power_number_right]+dynamixel.power_left[power_number_left]+dynamixel.power_idle*2)*1.1

        voltage = battery.voltage_full[voltage_decider]

        current = power/voltage

        energy_used += current*voltage

        new_currentCharge -= current

    return energy_used


def calc_power_usage(dist_to_goal, dist_to_dock):

    time_to_goal = (dist_to_goal/robot_params.max_linvel_x)*0.8 + (dist_to_goal/(0.5*robot_params.max_linvel_x))*0.2

    time_to_dock = (dist_to_dock/robot_params.max_linvel_x)*0.8 + (dist_to_dock/(0.5*robot_params.max_linvel_x))*0.2

    power_usage = [0]*3

    power_usage[0] = power_to_goal(time_to_goal)

    power_usage[1] = power_to_dock(time_to_dock,power_usage[0][1] )

    power_usage[2] = power_usage[0][0] + power_usage[1]

    return power_usage


def callback_power_comp(capacity):

    charge.current = capacity.data


def callback_path(path):

    if travel_dist.counter < 1:

        travel_dist.to_goal_initial = path_distance(path, increment_gain = 1)

        travel_dist.counter += 1

    else:

        travel_dist.to_goal = path_distance(path, increment_gain = 1)

        travel_dist.counter += 1


def callback_goal(goal):

    charging_station = load_charging_station(coord_x = -2.32499957085, coord_y = 1.19209289551e-07)

    get_path(goal.goal.target_pose, charging_station, tol = 1)


if __name__ == '__main__':

    # Initializing

    rospy.init_node("path_power_predicter_real_node")

    rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, callback_goal)

    rospy.Subscriber("move_base/NavfnROS/plan", Path, callback_path)

    rospy.Subscriber("/battery_charge", Float32, callback_power_comp)

    rospy.Subscriber("/move_base/status", GoalStatusArray, callback_goal_status)

    pub = rospy.Publisher('/status_path', String, queue_size=1)


    rospy.spin()