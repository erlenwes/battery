#! /usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan, GetPlanRequest
from nav_msgs.msg import Path


rospy.init_node('test_node')

battery_station = PoseStamped()
start = PoseStamped()

battery_station.header.seq = 0
battery_station.header.stamp = rospy.Time.now()
battery_station.header.frame_id = "map"
battery_station.pose.position.x = -2.32499957085
battery_station.pose.position.y = 1.19209289551e-07
battery_station.pose.position.z = 0.0
battery_station.pose.orientation.x = 0.0
battery_station.pose.orientation.y = 0.0
battery_station.pose.orientation.z = -1.38615431446e-07
battery_station.pose.orientation.w = 1.0

start.header.seq = 0
start.header.stamp = rospy.Time.now()
start.header.frame_id = "map"
start.pose.position.x = 0.814999461174
start.pose.position.y = -0.600000143051
start.pose.position.z = 0.0
start.pose.orientation.x = 0.0
start.pose.orientation.y = 0.0
start.pose.orientation.w = 1.0


def service_start():

    rospy.wait_for_service('/move_base/NavfnROS/make_plan')

    connection_service = rospy.ServiceProxy('/move_base/NavfnROS/make_plan', GetPlan)

    param = GetPlanRequest()

    param.start = start
    param.goal = battery_station
    param.tolerance = 1

    result = connection_service(param)

    print(result.plan.poses)

service_start()