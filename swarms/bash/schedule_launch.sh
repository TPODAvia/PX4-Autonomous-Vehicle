#!/bin/bash
# Make the script executable with:

#       chmod +x run_ros_launch.sh
#
#To do this, open the crontab editor:

#       crontab -e

# This will run the script every 5 minutes (*/5), all hours, from the 1st to the 31st 
# day of the month, only in August (8), and only on Mondays and Fridays (1,5).

#       */5  * 1-31 8 1,5 /path/to/your/run_ros_launch.sh

#  * * * * *  command to execute
#  ┬ ┬ ┬ ┬ ┬
#  │ │ │ │ │
#  │ │ │ │ │
#  │ │ │ │ └───── day of the week (0-7) (0 to 6 are Sunday to Saturday, or use names; 7 is Sunday, the same as 0)
#  │ │ │ └────────── month (1-12)
#  │ │ └─────────────── day of the month (1-31)
#  │ └──────────────────── hour (0-23)
#  └───────────────────────── min (0-59)

# Get the current month
current_month=$(date '+%m')

# Check if the current month is August (08)
if [ "$current_month" == "08" ]; then
    # Run the ROS launch file
    roslaunch your_package your_launch_file.launch
fi