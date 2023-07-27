To enable the watchdog timer on your Raspberry Pi running Ubuntu 22.04, you can follow the steps below:

1. Activate the hardware watchdog on your Raspberry Pi. In your `config.txt` file, add the following line to enable the watchdog:

```bash
dtparam=watchdog=on
```
You can edit this file using the nano editor with the command `sudo nano /boot/config.txt`, then add the line above at the end of the file and save it [Source 7](https://medium.com/@arslion/enabling-watchdog-on-raspberry-pi-b7e574dcba6b).

2. Reboot your Raspberry Pi. After the reboot, you can check if the watchdog is enabled by running the command `ls -al /dev/watchdog*`. You should see output similar to this:

```bash
crw------- 1 root root  10, 130 May 19 07:09 /dev/watchdog
crw------- 1 root root 253,   0 May 19 07:09 /dev/watchdog0
```
This indicates that the watchdog is enabled [Source 7](https://medium.com/@arslion/enabling-watchdog-on-raspberry-pi-b7e574dcba6b).

3. Install the watchdog software package with the command `sudo apt-get install watchdog` [Source 1](https://forums.raspberrypi.com//viewtopic.php?t=147501).

4. Configure the watchdog package. Edit the configuration file `/etc/watchdog.conf` and uncomment or add the following lines:

```bash
max-load-1 = 24
min-memory = 1
watchdog-device = /dev/watchdog
watchdog-timeout=15
```
You can edit this file using the nano editor with the command `sudo nano /etc/watchdog.conf`, then make the changes and save the file [Source 7](https://medium.com/@arslion/enabling-watchdog-on-raspberry-pi-b7e574dcba6b).

5. Start the watchdog service with the command `sudo systemctl start watchdog` and enable it to start at boot with the command `sudo systemctl enable watchdog` [Source 7](https://medium.com/@arslion/enabling-watchdog-on-raspberry-pi-b7e574dcba6b).

6. Verify that the watchdog service is running with the command `sudo systemctl status watchdog`. If it's running correctly, you should see output indicating that the service is active [Source 1](https://forums.raspberrypi.com//viewtopic.php?t=147501).

With these steps, you should have successfully enabled the watchdog timer on your Raspberry Pi running Ubuntu 22.04. The watchdog timer will help to automatically reboot the system in case it freezes.
