# LED strip support for ROS

## led_msgs package

Common messages for LEDs.

## ws281x package

ws281x LED strip driver for ROS (for a Raspberry Pi only for now). Based on the [rpi_ws281x library](https://github.com/jgarff/rpi_ws281x).

### Running without root permissions

To allow running the node without root permissions set the [`setuid`](https://en.wikipedia.org/wiki/Setuid) bit to the executable:

```bash
sudo chown root:root $(catkin_find ws281x ws281x_node)
sudo chmod +s $(catkin_find ws281x ws281x_node)
```