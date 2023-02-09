import roslaunch
import rospy
import time

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/vboxuser/test_ws/src/learning_tf2/launch/save_map.launch"])
launch.start()
try:
  launch.spin()
  time.sleep(60)
finally:
  launch.shutdown()