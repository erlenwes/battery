#! /usr/bin/env python
import rospy
from nav_msgs.msg import Path
from math import sqrt
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.srv import GetPlan, GetPlanRequest
from std_msgs.msg import Float32
from actionlib_msgs.msg import GoalStatus
from actionlib_msgs.msg import GoalStatusArray
import numpy


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

class charge:
    joule = 0
    maximum = 1.8*11.1*3600
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
    if ((goal_status.status_list[len(goal_status.status_list)-1].status) == 3 and (charge.status_before == 1)):
        charge.status_before = (goal_status.status_list[len(goal_status.status_list)-1].status)
        print("Actual battery usage: {} %".format(round(100*(charge.before-charge.after)/charge.maximum,3)))
    #If current status is goal reached just update charge_before
    if (goal_status.status_list[len(goal_status.status_list)-1].status == 3):
        charge.before = charge.joule
        charge.status_before = (goal_status.status_list[len(goal_status.status_list)-1].status)
    #If current status is moving then update charge.after until the new status is goal reached
    if (goal_status.status_list[len(goal_status.status_list)-1].status) == 1:
        charge.after = charge.joule
        charge.status_before = (goal_status.status_list[len(goal_status.status_list)-1].status)


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
    print("-------------------------------------------------------------")
    print("Initial distance to goal: {} m".format(round(travel_dist.to_goal_initial,3)))
    print("Dist to battery station: {} m".format(round(travel_dist.back_to_station,3)))
    print("Estimated battery required for safe operation: {}%".format(round(100*eng_path[2]/charge.maximum,2)))
    print("Estimated battery required for next task: {} %".format(round(100*eng_path[0]/charge.maximum,2)))

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



def calc_power_usage(dist_to_goal, dist_to_station):

    power_number_right = int(robot_params.max_linvel_x*100)
    power_number_left = power_number_right

    time_to_goal = dist_to_goal/robot_params.max_linvel_x
    time_to_station = dist_to_station/robot_params.max_linvel_x

    power_usage = [0]*3

    power_usage[0] = time_to_goal*(mcu.power+(camera.on*camera.power)+raspi.power_b+raspi.power_bp+sensor.power+dynamixel.power_right[power_number_right]+dynamixel.power_left[power_number_left]+dynamixel.power_idle*2)
    power_usage[1] = time_to_station*(mcu.power+(camera.on*camera.power)+raspi.power_b+raspi.power_bp+sensor.power+dynamixel.power_right[power_number_right]+dynamixel.power_left[power_number_left]+dynamixel.power_idle*2)
    power_usage[2] = power_usage[0] + power_usage[1]

    #print("Estimated time to goal: {}".format(time_to_goal))
    #print("Estimated time to station from goal: {}".format(time_to_station))
    #print("Estimated energy usage to goal: {} J".format(power_usage[2]))
    #print("Estimated energy usage to station from goal: {} J".format(power_usage[1]))

    return power_usage


def callback_power_comp(energy):

    charge.joule = energy.data


def callback_path(path):

    if travel_dist.counter < 1:
        travel_dist.to_goal_initial = path_distance(path, increment_gain = 1)
        travel_dist.counter += 1
    else:
        travel_dist.to_goal = path_distance(path, increment_gain = 1)
        #print("Current distance to goal: " +str(travel_dist.to_goal))
        travel_dist.counter += 1


def callback_goal(goal):

    charging_station = load_charging_station(coord_x = -2.32499957085, coord_y = 1.19209289551e-07)

    get_path(goal.goal.target_pose, charging_station, tol = 1)


if __name__ == '__main__':

    # Initializing

    rospy.init_node("power_predicter_node")

    rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, callback_goal)
    rospy.Subscriber("move_base/NavfnROS/plan", Path, callback_path)
    rospy.Subscriber("/battery_charge", Float32, callback_power_comp)
    rospy.Subscriber("/move_base/status", GoalStatusArray, callback_goal_status)


    rospy.spin()