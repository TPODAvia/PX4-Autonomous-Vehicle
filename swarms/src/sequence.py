import rospy
import time
import os
import mavros
from mavros.utils import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String
import re
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import State
# https://github.com/Mohit505Git/Mavros-AUTO.MISSION-mode/blob/master/wayPointMission.py
# http://docs.ros.org/api/mavros_msgs/html/msg/Waypoint.html
# Check the https://github.com/mavlink/mavros/blob/master/mavros_msgs/msg/CommandCode.msg
# Initialize global variables
available_drones = set()
mavros.set_namespace()

class px4FlightMode:

    def __init__(self, drone_name):
        self.drone_name = drone_name
        pass
    
    def setTakeoff(self):
        rospy.wait_for_service(self.drone_name + '/mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy(self.drone_name + '/mavros/cmd/takeoff',mavros_msgs.srv.CommandTOL)
            takeoffService(Altitude = 2.5)
            rospy.loginfo("Taking off")
        except rospy.ServiceException as e:
            rospy.logerror("Takeoff failed: %s"%e)

    def setAutoLandMode(self):
        rospy.wait_for_service(self.drone_name + '/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy(self.drone_name + '/mavros/set_mode',mavros_msgs.srv.SetMode)
            flightModeService(custom_mode = 'AUTO.LAND')
            rospy.loginfo("Landing")
        except rospy.ServiceException as e:
            rospy.logerror("Landing failed: %s. Autoland Mode could not be set"%e)

    def setArm(self):
        rospy.wait_for_service(self.drone_name + '/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy(self.drone_name + '/mavros/cmd/arming',mavros_msgs.srv.CommandBool)
            armService(True)
            rospy.loginfo("Arming motors OK")
        except rospy.ServiceException as e:
            rospy.logerror("Arming motors is failed: %s"%e)

    def clearMission(self):
        os.system(f"rosrun mavros mavwp -n {self.drone_name}/mavros -v clear")
        rospy.loginfo("Mission WayPoint Cleared!")

    def loadMission(self):
        os.system(f"rosrun mavros mavwp -n {self.drone_name}/mavros -v load ~/catkin_ws/src/px4_ground/mission/missionwp_land.txt")
        rospy.loginfo("Mission WayPoint Loaded!")
        os.system(f"rosrun mavros mavwp -n {self.drone_name}/mavros -v show")
    
    def setAutoMissionMode(self):
        rospy.wait_for_service(self.drone_name + "/mavros/set_mode")
        try:
            flightModeService = rospy.ServiceProxy(self.drone_name + '/mavros/set_mode',mavros_msgs.srv.SetMode)
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


my_mission_success = ""
current_state = State()
def state_cb(msg):
    global current_state
    global my_mission_success
    current_state = msg
    my_mission_success = current_state.mode


VOLTAGE_CAPACITY = 0.0
CRITICAL_VOLTAGE = 8.0
trigger_state = False
def battery_state_cb(msg):
    # rospy.loginfo("Battery voltage: %f V", msg.voltage)
    global VOLTAGE_CAPACITY
    currect_voltage = msg.voltage
    if currect_voltage > VOLTAGE_CAPACITY:
        VOLTAGE_CAPACITY = currect_voltage

    if (VOLTAGE_CAPACITY - (VOLTAGE_CAPACITY - currect_voltage)*2) < CRITICAL_VOLTAGE:
        rospy.logerr("Battery critical voltage reached")
        global trigger_state
        trigger_state = True
    else:
        trigger_state = False

words = []
last_received_status = {}
def message_callback(msg):
    global available_drones
    words = re.split(r'\s+', msg.data.strip())

    vector = [int(words[1]), words[2] == "in_mission", words[3] == "reached"]
    drones_vector = tuple(vector)
    available_drones = {drone for drone in available_drones if drone[0] != drones_vector[0]}

    # Add the new drone with updated status values
    if drones_vector:
        available_drones.add(drones_vector)

    # Sort the available_drones
    available_drones = sorted(available_drones)
    # print("Drones available", available_drones)
    # Store the last received status for the drone
    last_received_status[int(words[1])] = time.time()



def main():

    rospy.init_node('swarm_sequence_node',anonymous=True)
    my_drone_id = rospy.get_param('~drone_id', -1) 
    my_drone_name = rospy.get_param('~drone_name', "drones") + str(my_drone_id)
    PX4modes = px4FlightMode(my_drone_name)
    my_drone_id_ready_pub = rospy.Publisher("/drone_id_ready", String, queue_size=10)
    swarm_data = String()

    PX4modes.clearMission()
    PX4modes.loadMission()
    # Create a publisher and subscriber
    my_drone_id_ready_sub = rospy.Subscriber("/drone_id_ready", String, message_callback)
    sub = rospy.Subscriber(my_drone_name + '/mavros/mission/reached',WaypointReached, WP_callback)
    battery_sub = rospy.Subscriber(my_drone_name + "/mavros/battery", BatteryState, battery_state_cb)
    state_sub = rospy.Subscriber(my_drone_name + "/mavros/state", State, state_cb)
    # drone_init = rospy.Subscriber(drone_name + i +'mavros/mission/reached',WaypointReached,check_drones_init)
    # Create a String message
    rate = rospy.Rate(2)

    global words
    global trigger_state
    global my_mission_success

    for i in range(5):
        # if(rospy.is_shutdown()):
        #     break
        swarm_data.data = f"Drone {my_drone_id} not_in_mission not_reached"
        my_drone_id_ready_pub.publish(swarm_data)
        rate.sleep()
        
    while(not rospy.is_shutdown()):

        # Check for drone timeout and remove it if necessary
        # print("Drones available", available_drones)
        for drone_id, last_seen in list(last_received_status.items()):
            current_time = time.time()
            if current_time - last_seen > 2:
                for drone in available_drones:
                    if drone[0] == drone_id:
                        available_drones.remove(drone)
                        del last_received_status[drone_id]
                        break
        
        print("Drones available", available_drones)
        swarm_data.data, flight_trigger = caller(my_drone_id)
        # if str(my_drone_id) == "0":
        #     swarm_data.data = f"Drone {my_drone_id} in_mission reached"
        my_drone_id_ready_pub.publish(swarm_data)
        if flight_trigger:
            PX4modes.setAutoMissionMode()
            #PX4modes.setTakeoff()
            PX4modes.setArm()
        rate.sleep()

def caller(my_drone_id):
    drone_ids = []
    for drone_id, in_mission, trigger in available_drones:
        if in_mission:
            if drone_id == my_drone_id:
                # print(f"Drone {my_drone_id} in mission trigger when needed")
                if my_mission_success in ("RETURN", "AUTO.RTL") or trigger_state:
                    swarm_data = f"Drone {my_drone_id} in_mission reached"
                
                elif my_mission_success in ("LANDING", "AUTO.LAND"):
                    swarm_data = f"Drone {my_drone_id} not_in_mission not_reached"
                else:
                    swarm_data = f"Drone {my_drone_id} in_mission not_reached"

                return swarm_data, False
            
            if trigger:
                drone_ids = []
                for drone_id, in_mission, trigger in available_drones:
                    if not in_mission:
                        drone_ids.append(drone_id)
                    if in_mission and not trigger and drone_id != my_drone_id:
                        print("Hello1")
                        swarm_data = f"Drone {my_drone_id} not_in_mission not_reached"
                        return swarm_data, False
                    
                    elif in_mission and not trigger and drone_id == my_drone_id:
                        print("Hello12")
                        swarm_data = f"Drone {my_drone_id} in_mission not_reached"
                        return swarm_data, False
                    
                count = sum(1 for drone_id in drone_ids if drone_id < my_drone_id)
                print(f"Number of drone_ids smaller than {my_drone_id}: {count}")

                # Check the priority of the drone
                if count == 0:
                    print(f"Count is: {my_drone_id}")
                    swarm_data = f"Drone {my_drone_id} in_mission not_reached"
                    return swarm_data, True
                
            print("Hello2")
            # print(f"Drone {my_drone_id} not in mission and not reached")
            swarm_data = f"Drone {my_drone_id} not_in_mission not_reached"
            return swarm_data, False
        drone_ids.append(drone_id)

    count = sum(1 for drone_id in drone_ids if drone_id < my_drone_id)

    print(f"Sorted drone_ids: {drone_ids}")
    print(f"Number of drone_ids smaller than {my_drone_id}: {count}")

    # Check the priority of the drone
    if count == 0:
        print(f"Count is 0 for the drone {my_drone_id}")
        swarm_data = f"Drone {my_drone_id} in_mission not_reached"
        return swarm_data, True
    else:
        print("Hello3")
        print(f"Drone {my_drone_id} not in mission and not reached")
        swarm_data = f"Drone {my_drone_id} not_in_mission not_reached"
        return swarm_data, False

if __name__ == '__main__':
        try:
                main()
        except rospy.ROSInterruptException:
                pass
