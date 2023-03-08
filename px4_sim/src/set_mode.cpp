/***************************************************************************************************************************
* set_mode.cpp
*
* Author: Qyp
*
* Update Time: 2019.3.16
*
* Introduction:  Change mode
***************************************************************************************************************************/

// head File
#include <ros/ros.h>


#include <iostream>

// topic header file
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


using namespace std;
mavros_msgs::State current_state;                       // The current state of the drone [including the locked state mode] (read from the flight control)
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Callback<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>main function<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_mode");
    ros::NodeHandle nh("~");

    // 【Subscription】Current Status of UAV - From Flight Control
    //  This topic comes from flight control (via /plugins/sys_status.cpp)
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);

    double mode = -1;
    nh.getParam("output_mode", mode);
    cout << "default_mode" << mode << endl;


    // 【Service】Modify system mode
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    mavros_msgs::SetMode mode_cmd;

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Rate rate(10.0);

    int Num_StateMachine = 0;
    int flag_1;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>main loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        switch (Num_StateMachine)
        {
            // input
            case 0:
                cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>--------<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
                cout << "Input the mode:  0 for Arm,1 for TAKEOFF, 2 for OFFBOARD,3 for LAND, 4 for POSCTL,5 for MISSION  "<<endl;
                if (mode != -1)
                {
                    flag_1 = mode;
                }
                else
                {
                    cin >> flag_1;
                }
 
                //1000 Landing can also point to other quests
                if (flag_1 == 0)
                {
                    Num_StateMachine = 1;
                    break;
                }
                else if (flag_1 == 1)
                {
                    Num_StateMachine = 2;
                    break;
                }
                else if(flag_1 == 2)
                {
                    Num_StateMachine = 3;
                }//Inertial frame movement
                else if(flag_1 == 3)
                {
                    Num_StateMachine = 4;
                }
                else if(flag_1 == 4)
                {
                    Num_StateMachine = 5;
                }
                else if(flag_1 == 5)
                {
                    Num_StateMachine = 6;
                }

                break;

        //Arm
        case 1:
           if(!current_state.armed)
            {
                arm_cmd.request.value = true;
                arming_client.call(arm_cmd);

                cout << "Arming..." <<endl;

            }else
            {
                Num_StateMachine = 0;
                cout << "Arm Susscess!!!" <<endl;
            }
            break;

        //TAKEOFF
        case 2:
            if(current_state.mode != "AUTO.TAKEOFF")
            {
                mode_cmd.request.custom_mode = "AUTO.TAKEOFF";
                set_mode_client.call(mode_cmd);
                cout << "Setting to TAKEOFF Mode..." <<endl;

            }else
            {
                Num_StateMachine = 0;
                cout << "Set to AUTO.TAKEOFF Mode Susscess!!!" <<endl;
            }
            break;

        //OFFBOARD
        case 3:
            if(current_state.mode != "OFFBOARD")
            {
                mode_cmd.request.custom_mode = "OFFBOARD";
                set_mode_client.call(mode_cmd);
                cout << "Setting to OFFBOARD Mode..." <<endl;

            }else
            {
                Num_StateMachine = 0;
                cout << "Set to OFFBOARD Mode Susscess!!!" <<endl;
            }
            break;

        //LAND
        case 4:
            if(current_state.mode != "AUTO.LAND")
            {
                mode_cmd.request.custom_mode = "AUTO.LAND";
                set_mode_client.call(mode_cmd);
                cout << "Setting to LAND Mode..." <<endl;

            }else
            {
                Num_StateMachine = 0;
                cout << "Set to LAND Mode Susscess!!!" <<endl;
            }
            break;

        //POSCTL
        case 5:
            if(current_state.mode != "POSCTL")
            {
                mode_cmd.request.custom_mode = "POSCTL";
                set_mode_client.call(mode_cmd);
                cout << "Setting to POSCTL Mode..." <<endl;

            }else
            {
                Num_StateMachine = 0;
                cout << "Set to POSCTL Mode Susscess!!!" <<endl;
            }
            break;

        //MISSION
        case 6:
            if(current_state.mode != "AUTO.MISSION")
            {
                mode_cmd.request.custom_mode = "AUTO.MISSION";
                set_mode_client.call(mode_cmd);
                cout << "Setting to MISSION Mode..." <<endl;

            }else
            {
                Num_StateMachine = 0;
                cout << "Set to RTL MISSION Susscess!!!" <<endl;
            }
            break;
        }

        //Execute the callback function
        ros::spinOnce();
        //Cycle sleep
        rate.sleep();
    }

    return 0;

}
