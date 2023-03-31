#!/usr/bin/env python3

########################################################################################
# AMOV
# px4+mavros:load a mission using waypoints file(.txt),don't support .plan format file
# author: Eason Yi
# email:  eason473867143@gmail.com
# time:   04/08/2020
# v1.0
########################################################################################

import rospy
import time
import os
import mavros
from mavros import command
from mavros.utils import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *

mavros.set_namespace()

class px4FlightMode:

    def __init__(self):
        pass
    
    def setTakeoff(self):
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff',mavros_msgs.srv.CommandTOL)
            takeoffService(Altitude = 2.5)
            rospy.loginfo("Taking off")
        except rospy.ServiceException as e:
            rospy.logerror("Takeoff failed: %s"%e)

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode',mavros_msgs.srv.SetMode)
            flightModeService(custom_mode = 'AUTO.LAND')
            rospy.loginfo("Landing")
        except rospy.ServiceException as e:
            rospy.logerror("Landing failed: %s. Autoland Mode could not be set"%e)

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming',mavros_msgs.srv.CommandBool)
            armService(True)
            rospy.loginfo("Arming motors OK")
        except rospy.ServiceException as e:
            rospy.logerror("Arming motors is failed: %s"%e)

    def clearMission(self):
        os.system("rosrun mavros mavwp clear")
        rospy.loginfo("Mission WayPoint Cleared!")

    def loadMission(self):
        os.system("rosrun mavros mavwp load ~/catkin_ws/src/px4_ground/mission/missionwp_land.txt")
        rospy.loginfo("Mission WayPoint Loaded!")
        os.system("rosrun mavros mavwp show")
    
    def setAutoMissionMode(self):
        rospy.wait_for_service("mavros/set_mode")
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode',mavros_msgs.srv.SetMode)
            flightModeService(custom_mode = 'AUTO.MISSION')
            rospy.loginfo("Entering Auto Mission Mode OK!")
        except rospy.ServiceException as e:
            rospy.logerror("Entering Auto Mission Mode failed: %s. AUTO.MISSION mode could not be set."%e)

    def read_failsafe(self):
        try:
            get = rospy.ServiceProxy(mavros.get_topic('param','get'),ParamGet)
            DLL_param = get(param_id = "NAV_DLL_ACT")
            RCL_param = get(param_id = "NAV_RCL_ACT")
            print( " ")
            print( "-------failsafe status----------")
            print( "Present NAV_DLL_ACT value is", DLL_param.value.integer)
            print( "Present NAV_RCL_ACT value is", RCL_param.value.integer)
            print( "--------------------------------")
            print( " ")
            return {'DL':DLL_param.value.integer,'RC':RCL_param.value.integer}
        except rospy.ServiceException as e:
            rospy.logerror("failsafe status read failed: %s"%e)

    def remove_failsafe(self):
        try:
            val = ParamValue(integer=2,real=0.0)
            set = rospy.ServiceProxy(mavros.get_topic('param','set'), ParamSet)
            new_DLL_param = set(param_id = "NAV_DLL_ACT", value = val)
            new_RCL_param = set(param_id = "NAV_RCL_ACT", value = val)
            print( " ")
            print( "-------remove failsafe----------")
            print( "New NAV_DLL_ACT value is", new_DLL_param.value.integer)
            print( "New NAV_RCL_ACT value is", new_RCL_param.value.integer)
            print( "--------------------------------")
            print( " ")
        except rospy.ServiceException as e:
            rospy.logerror("failsafe status change failed: %s"%e)

def WP_callback(msg):

    global last_wp
    global starting_time
    global mission_started

    try:
        mission_started
    except NameError:
        rospy.loginfo("starting Mission WayPoint #0 reached")
        starting_time = msg.header.stamp.secs
        mission_started = True
    else:
        if msg.wp_seq == 0 and msg.wp_seq != last_wp:
            elapsed_time = msg.header.stamp.secs - starting_time
            rospy.loginfo("Ending Mission: Total time: %d s",elapsed_time)
        elif msg.wp_seq != last_wp:
            elapsed_time = msg.header.stamp.secs - starting_time
            rospy.loginfo("Mission WayPoint #%s reached. Elasped time: %d s",msg.wp_seq,elapsed_time)

    last_wp = msg.wp_seq

    
def main():

    rospy.init_node('mavros_mission_node',anonymous=True)

    PX4modes = px4FlightMode()

    #failsafe_status = PX4modes.read_failsafe()
    #if(failsafe_status['DL'] != 0) or (failsafe_status['RC' != 0]):
    #    PX4modes.remove_failsafe()

    PX4modes.clearMission()

    PX4modes.loadMission()
    PX4modes.setAutoMissionMode()
    #PX4modes.setTakeoff()
    PX4modes.setArm()

    sub = rospy.Subscriber('mavros/mission/reached',WaypointReached,WP_callback)

    rospy.spin()

if __name__ == '__main__':
        try:
                main()
        except rospy.ROSInterruptException:
                pass
