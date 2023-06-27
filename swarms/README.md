# Swarms

The code in this repo is explains how to set up your drone swarms systems as well as teaches basic drone programming commands. 

This project need to use WiFi hotspot for drone collaboration. Make sure that network is properly setup:

[Setting_Hotspot](docs/Setting_Hotspot.md)

### Non-stop delivery of goods using multiple drones

The small payload of drones is a big disadvantage. This task solves the logistics of transportation of small, but large volumes of goods. This is achieved through the collaboration of several drones and can significantly reduce and speed up the delivery time of the cargo.

```bash
roslaunch swarms drones_sequence0.launch
```

### Simple group flight for the simultaneous delivery of goods

This task solves a way to prevent collision between several drones while flying on missions. The minimum distance between drones is at least 5 meters (GPS accuracy limits). For coordinated work, the concept of a master-slave drone is used. The lead drone in this case is a Wi-Fi distributor for other drones. The maximum number of drones is 8. (Restrictions on the hotspot of Wi-Fi)

```bash
roslaunch swarms multi_square_sol0.launch
```
