import rospy
import time
import os
import mavros
from mavros import command
from mavros.utils import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String
import re

# Initialize global variables
ready_drones_count = 0
available_drones = set()
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

    rospy.init_node('swarm_sequence_node',anonymous=True)

    PX4modes = px4FlightMode()

    #failsafe_status = PX4modes.read_failsafe()
    #if(failsafe_status['DL'] != 0) or (failsafe_status['RC' != 0]):
    #    PX4modes.remove_failsafe()

    PX4modes.clearMission()

    PX4modes.loadMission()
    PX4modes.setAutoMissionMode()
    #PX4modes.setTakeoff()
    PX4modes.setArm()

    # Create a String message
    swarm_data = String()
    swarm_data.data = "Drone 0 Leader Reached"

    # Create a publisher and subscriber
    my_drone_id_ready_pub = rospy.Publisher("/drone_id_ready", String, queue_size=10)
    my_drone_id_ready_sub = rospy.Subscriber("/drone_id_ready", String, message_callback)
    sub = rospy.Subscriber('mavros/mission/reached',WaypointReached,WP_callback)
    # drone_init = rospy.Subscriber(drone_name + i +'mavros/mission/reached',WaypointReached,check_drones_init)
    # Publish the message

    rate = rospy.Rate(20)
    while(not rospy.is_shutdown()):
        # loop every drones and check if they are running
        #   If (once running):
        #         If (check the trigger):
        #            check the drone prority
        #            If (it is my turn):
        #                  start the mission               
        #   Else 
        #         check the drone prority
        #         if (it is my turn):
        #               start the mission
        my_drone_id_ready_pub.publish(swarm_data)
        rate.sleep()

# first - in mission
# second - ready
# third - trigger
def message_callback(msg):
    words = re.split(r'\s+', msg.data.strip())

    drone_id = int(words[1])
    leader_status = words[2]
    reached_status = words[3]

    # If the drone is a leader, print its ID
    if leader_status == "Leader":
        rospy.loginfo("Leader drone: %d", drone_id)

    # If the drone reached its destination, print its ID
    if reached_status == "Reached":
        rospy.loginfo("Drone %d reached its destination.", drone_id)

    # If the drone reached its destination, increment the ready counter and add the drone to the available set
    if reached_status == "Reached":
        global ready_drones_count
        ready_drones_count += 1
        available_drones.add(drone_id)

    rospy.loginfo("Ready drones count: %d", ready_drones_count)
    rospy.loginfo("Available drones: ")

    for drone in available_drones:
        rospy.loginfo("Drone %d", drone)



if __name__ == '__main__':
        try:
                main()
        except rospy.ROSInterruptException:
                pass
