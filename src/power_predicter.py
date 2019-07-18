#! /usr/bin/env python
import rospy
from nav_msgs.msg import Path
from math import sqrt
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.srv import GetPlan, GetPlanRequest
from std_msgs.msg import Float32


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

    #rospy.wait_for_service('/move_base/NavfnROS/make_plan')
    travel_dist.counter = 0
    connection_service = rospy.ServiceProxy('/move_base/NavfnROS/make_plan', GetPlan)

    param = GetPlanRequest()

    param.start = start
    param.goal = goal
    param.tolerance = tol

    path = connection_service(param)
    travel_dist.back_to_station = path_distance(path.plan, increment_gain = 1)
    print("Dist to battery station: " +str(round(travel_dist.back_to_station,3))+"m")

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


def callback_power_comp(charge):

    

def callback_path(path):

    if travel_dist.counter < 1:
        travel_dist.to_goal_initial = path_distance(path, increment_gain = 1)
        print("Initial distance to goal: " +str(round(travel_dist.to_goal_initial,3))+ "m")
        travel_dist.counter += 1
    else:
        travel_dist.to_goal = path_distance(path, increment_gain = 1)
        #print("Current distance to goal: " +str(travel_dist.to_goal))
        travel_dist.counter += 1


def callback_goal(goal):

    charging_station = load_charging_station(coord_x = -2.32499957085, coord_y = 1.19209289551e-07)

    get_path(goal.goal.target_pose, charging_station, tol = 1)

class travel_dist:

    to_goal_initial = 0
    to_goal = 0
    back_to_station = 0
    counter = 0

class robot_params:

    max_linvel_x = (rospy.get_param('/move_base/DWAPlannerROS/max_vel_x')) #m/s
    max_linacc_x = (rospy.get_param('/move_base/DWAPlannerROS/acc_lim_x'))  #m/s^2
    max_angvel_z = (rospy.get_param('/move_base/DWAPlannerROS/max_rot_vel')) #rad/s

if __name__ == '__main__':

    # Initializing

    rospy.init_node("power_predicter_node")
    rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, callback_goal)
    rospy.Subscriber("move_base/NavfnROS/plan", Path, callback_path)
    rospy.Subscriber("/battery_charge", Float32, callback_power_comp)


    rospy.spin()