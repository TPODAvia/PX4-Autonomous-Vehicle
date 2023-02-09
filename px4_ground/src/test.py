
# NOTE: Manual Control in QGroundControl is out off this control loop

ManualCommand = True or False
ObsticalZone = True or False
TargetAreaLanding = True or False
LandingCommand = True or False

while 1:
    if ManualCommand == True:
        print("#Do Manual Control Over Wifi")
    else:
        if (LandingCommand == True):
            print("#landing or Return")
            print("#ROS.shutdown()")

        elif TargetAreaLanding == True:
            print("#Do Detection with SLAM and QGroundControl")

        elif ObsticalZone == True and TargetAreaLanding== False:
            print("#DO SLAM Control with Mission QGroundControl")