import schedule
import time
import subprocess
import rospy

launch_files = [
    ('weekly_launch', 'weekly_node.launch'),
    ('another_package', 'another_node.launch'),
    # Add more launch files here
]

def job():
    processes = []
    for package, launch_file in launch_files:
        rospy.loginfo(f'Starting {launch_file}')
        process = subprocess.Popen(['roslaunch', package, launch_file])
        processes.append(process)

    while not rospy.is_shutdown() and any(process.poll() is None for process in processes):
        time.sleep(1)

    for process in processes:
        if process.poll() is None:
            process.terminate()
            process.wait()
            rospy.loginfo(f'{launch_file} stopped')

schedule.every().monday.at("00:00").do(job)

while True:
    schedule.run_pending()
    time.sleep(1)
