#! /usr/bin/env python
import rospy
from battery_state.msg import BatteryState
from array import array

voltage_track = array['f']
current_track = array['f']
capacity = 0
design_capacity = 0
percentage = 0



def callback(msg)

    global voltage_track
    global current_track
    global capacity
    global design_capacity
    global percentage

    voltage_track.append(msg.voltage) #V
	current_track.append(msg.current) #A Negative when discharege
	capacity= msg.capacity #Ah Last full capacity
	design_capacity = msg.design_capacity #Ah Degisn capacity
	percentage = msg.percentage #Charge percentage from 0 to 1
    print(*voltage_track)
    print(*current_track)
    rate.sleep()

rospy.init_node('batteryTracker')
sub = rospy.Subscriber('sensor_msgs/BatteryState', BatteryState, callback)
ropsy.spin()