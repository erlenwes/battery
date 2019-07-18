#! /usr/bin/env python
import rospy
from nav_msgs.msg import Path
from math import sqrt
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.srv import GetPlan, GetPlanRequest
from std_msgs.msg import Float32
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
        current[dyna_counter] = (i*500/0.26)
        dyna_counter += 1

    #Idle power_dynamixcel
    power_idle = voltage*current_idle*0.001
    #Power calculation relative to speed
    for i in range(len(current)):
        power_right[i] = voltage*current[i]*0.001
        power_left[i] = voltage*current[i]*0.001

class charge:
    joule = 0


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


def get_path(start, goal, tol = 1):

    travel_dist.counter = 0
    rospy.wait_for_service('/move_base/NavfnROS/make_plan')
    connection_service = rospy.ServiceProxy('/move_base/NavfnROS/make_plan', GetPlan)

    param = GetPlanRequest()

    param.start = start
    param.goal = goal
    param.tolerance = tol

    path = connection_service(param)
    travel_dist.back_to_station = path_distance(path.plan, increment_gain = 1)
    print("Dist to battery station: " +str(round(travel_dist.back_to_station,3))+"m")
    num = calc_power_usage(travel_dist.to_goal_initial, travel_dist.back_to_station)
    print("This will use: "+str(round(100*num/charge.joule,2))+"%" +"of the remaining energy")

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

    print("Estimated time to goal: "+str(time_to_goal))
    print("Estimated time to station from goal: "+str(time_to_station))



    power_usage_goal = time_to_goal*(mcu.power+(camera.on*camera.power)+raspi.power_b+raspi.power_bp+sensor.power+dynamixel.power_right[power_number_right]+dynamixel.power_left[power_number_left]+dynamixel.power_idle*2)
    power_usage_station = time_to_station*(mcu.power+(camera.on*camera.power)+raspi.power_b+raspi.power_bp+sensor.power+dynamixel.power_right[power_number_right]+dynamixel.power_left[power_number_left]+dynamixel.power_idle*2)
    power_usage_tot = power_usage_goal + power_usage_station

    print("Estimated energy usage to goal: "+str(power_usage_goal)+"J")
    print("Estimated energy usage to station from goal: "+str(power_usage_station)+"J")

    return power_usage_tot


def callback_power_comp(energy):

    charge.joule = energy.data


def callback_path(path):

    if travel_dist.counter < 1:
        travel_dist.to_goal_initial = path_distance(path, increment_gain = 1)
        print("-------------------------------------------------------------------")
        print("Initial distance to goal: " +str(round(travel_dist.to_goal_initial,3))+ "m")
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


    rospy.spin()