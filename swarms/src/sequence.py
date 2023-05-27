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

PX4modes = px4FlightMode()
my_drone_id_ready_pub = rospy.Publisher("/drone_id_ready", String, queue_size=10)
swarm_data = String()
def main():

    rospy.init_node('swarm_sequence_node',anonymous=True)

    global my_drone_id
    my_drone_id = rospy.get_param('~drone_id', 0) 

    # PX4modes.clearMission()

    # Create a publisher and subscriber
    my_drone_id_ready_sub = rospy.Subscriber("/drone_id_ready", String, message_callback)
    sub = rospy.Subscriber('mavros/mission/reached',WaypointReached, WP_callback)
    # drone_init = rospy.Subscriber(drone_name + i +'mavros/mission/reached',WaypointReached,check_drones_init)
    # Create a String message
    rate = rospy.Rate(2)

    for i in range(10):
        if(rospy.is_shutdown()):
            break

        swarm_data.data = f"Drone {my_drone_id} not_in_mission not_reached"
        my_drone_id_ready_pub.publish(swarm_data)
        rate.sleep()


    while(not rospy.is_shutdown()):

        # swarm_data.data = f"Drone {my_drone_id} not_in_mission not_reached"
        # my_drone_id_ready_pub.publish(swarm_data)
        rate.sleep()

first_init = True
first_init_counter = 0
my_mission_success = False # This need to be changed
trigger_state = False # This need to be changed
def message_callback(msg):
    words = re.split(r'\s+', msg.data.strip())

    drone_id = int(words[1])
    in_mission_status = words[2]
    reached_status = words[3]

    # first - in mission
    # second - ready
    # third - trigger
    vector = [drone_id,
              in_mission_status == "in_mission",
              reached_status == "reached"]
    drones_vector = tuple(vector)

    # Check if a drone with the same drone_id exists
    existing_drone = None
    for drone in available_drones:
        if drone[0] == drone_id:
            existing_drone = drone
            break

    # If a drone with the same drone_id exists, remove it first
    if existing_drone:
        available_drones.remove(existing_drone)

    # Add the new drone with updated status values
    available_drones.add(drones_vector)
    # print("Drones available", available_drones)

    global first_init
    global first_init_counter
    if first_init:
        first_init_counter+=1
        if first_init_counter == 10:
            first_init = False
        return

    drone_ids = []
    for drone_id, in_mission, trigger in available_drones:
        if in_mission:
            if drone_id == my_drone_id:
                print(f"Drone {my_drone_id} in mission trigger when needed")

                if my_mission_success:
                    swarm_data.data = f"Drone {my_drone_id} not_in_mission not_reached"
                    my_drone_id_ready_pub.publish(swarm_data)
                    return

                if trigger_state:
                    swarm_data.data = f"Drone {my_drone_id} in_mission reached"
                    my_drone_id_ready_pub.publish(swarm_data)             
                return
            if trigger:
                drone_ids = []
                for drone_id, in_mission, trigger in available_drones:
                    if not in_mission:
                        drone_ids.append(drone_id)
                    if in_mission and not trigger:
                        swarm_data.data = f"Drone {my_drone_id} not_in_mission not_reached"
                        my_drone_id_ready_pub.publish(swarm_data)
                        return

                sorted_drone_ids = sorted(drone_ids)
                count = sum(1 for drone_id in sorted_drone_ids if drone_id < my_drone_id)

                # print(f"Sorted drone_ids: {sorted_drone_ids}")
                # print(f"Number of drone_ids smaller than {my_drone_id}: {count}")

                if count == 0:
                    print(f"Count is 0 {my_drone_id}")
                    swarm_data.data = f"Drone {my_drone_id} in_mission not_reached"
                    my_drone_id_ready_pub.publish(swarm_data)
                    # PX4modes.loadMission()
                    # PX4modes.setAutoMissionMode()
                    # #PX4modes.setTakeoff()
                    # PX4modes.setArm()
                    return
            print(f"Drone {my_drone_id} not in mission and not reached")
            swarm_data.data = f"Drone {my_drone_id} not_in_mission not_reached"
            my_drone_id_ready_pub.publish(swarm_data)
            return
        drone_ids.append(drone_id)
    
    sorted_drone_ids = sorted(drone_ids)
    count = sum(1 for drone_id in sorted_drone_ids if drone_id < my_drone_id)

    # print(f"Sorted drone_ids: {sorted_drone_ids}")
    # print(f"Number of drone_ids smaller than {my_drone_id}: {count}")

    if count == 0:
        print(f"Count is 0 for the drone {my_drone_id}")
        swarm_data.data = f"Drone {my_drone_id} in_mission not_reached"
        my_drone_id_ready_pub.publish(swarm_data)
        # PX4modes.loadMission()
        # PX4modes.setAutoMissionMode()
        # #PX4modes.setTakeoff()
        # PX4modes.setArm()



if __name__ == '__main__':
        try:
                main()
        except rospy.ROSInterruptException:
                pass
