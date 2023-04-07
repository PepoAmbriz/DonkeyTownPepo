# DonkieTown: a Low-cost Experimental Testbed for Research on Autonomous Cars
#### Abstract
*DonkieTown* is an affordable and scalable platform for research on autonomous vehicles. The experimental framework was developed in [ROS](https://www.ros.org/). The platform integrates multiple small scale autonomous vehicles called *Asinus Cars*, which are equipped with at least a camera, odometer, and onboard computer. The vehicles are Differential Drive Robots (DDR), forced by software to behave as car-like vehicles. DonkieTown incorporates a low-cost localization system to provide the real-time vehicles' pose, by means of external cameras which detect ArUco markers, then Kalman Filters (KF) are used to track and estimate the pose of each vehicle. The platform includes a base station computer with a graphical interface for monitoring the system. DonkieTown also includes a series of algorithms to facilitate autonomous driving, such as communication, tracking, object detection, obstacle avoidance, control, trajectory tracking, etc. Moreover, a centralized vehicular network is implemented to allow communication between the agents and the base station, where the agents can share information about their state, obstacles, maneuver intentions, etc.

![DonkieTown status](/docs/gifs/platoon.gif)

#### [Future] Cite and contribute
All developed source code, libraries and manufacturing files are released as open source under no license agreements. We expect every donkietown user to attribute our effort by citing *DonkieTown*.

```bibtex
@inproceedings{

}
```

To contribute, you are free to create, manage and mantain side branches. By the moment direct git pushes to main branch and any forced git push are not allowed, however, you may submit GitHub pull requests and the maintanance team will review them and decide whether to merge it or not.


## How to use
If you don't know how to install this project, visit [the installation page](/docs/INSTALL.md) for directions. 
### Base Station configuration
It is highly recommended to use a personal computer such as a laptop or workstation as the Base Station. The Base Station plays as ROS Master and Vehicular Network Manager. In order to setup your computer as ros_master and source simulator-only and base-station-only nodes you shall source:
```shell
source <DT_PATH>/bs_setup.bash
```
Permanently,
```shell
sudo sh -c "echo 'source <DT_PATH>/bs_setup.bash' >> ~/.bashrc"
```
once it is done, `$DT_PATH` environment variable is set and you may be able to start launching Simulator nodes, Navigation nodes, Localization Nodes, and Vehicular Communication Nodes.

### Simulator [Base station only]
Multiple simulation scenarios are provided in the bring_up package.
You can launch one scenario, e.g. in-house navigation, with the following command.
```shell
roslaunch bring_up navigation_inhouse.launch
```
![Simulator](/docs/gifs/simulation.gif)

### Asinus Cars
Please refer to [the jetson_nano page](/docs/jetson_nano/README.md)
![Asinus Car](/docs/images/AsinusCar.jpg)

### Upper cameras
Please refer to [the upper_cam page](/docs/upper_cam/README.md).

### ROS nodes and ROS launches
#### Asinus Car's *Core* node
Core node is the essential node of the asinus cars. This node enables ros topics to enable the motors, odometry and an Extended Kalman Filter. If you only want to control the asinus car, this node is the go. Also, alongside fake_gps node(s), you get an estimation of the robot's absolut position within the road.
```shell
roslaunch asinus_car core.launch car_id:=<marker_id>
```

#### Asinus Car's *Prime* node
In addition to *Core* features, *Prime* enables *DonkieNet* (a MobileNet+SSD detection network trained with a hand-labeled dataset of stuffed white donkeys).
```shell
roslaunch asinus_car prime.launch car_id:=<marker_id>
```

#### Vehicular communication node [Base Station only]
The following starts message handling and post-processing of shared data of pedestrians detected by all Asinus Cars.
```shell
roslaunch vehicular_communication network.launch
```

#### Shared data postprocessing (standalone) node [Base Station only]
```shell
roslaunch road_context obstacle_localization.launch
```

#### Navigation (standalone) node [Asinus car or Base Station]
It is recommended to use the base station as navigation controller since you could start it and finish it without login to the Asinus Car.
```shell
roslaunch fub_navigation navigation.launch car_id:=<marker_id>
```

#### Indoors localization system node [Base Station or upper cameras]
For this, you should calibrate each camera and recognize the port it is connected to. To calibrate a camera we recommend the [OpenCV's tutorial](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html).

To detect camera ports, connect one of your cameras and call the following command in the terminal:
```shell
ls /sys/dev/video*
```

or install and use v4l2 utils:
```shell
sudo apt-get install v4l-utils
v4l2-ctl --list-devices
```

Once you calibrated a camera and detect its port, launch the node.
```shell
roslaunch fake_gps fake_gps.launch upcam_id:=<upcam_id> cam_port:=<cam_port> calib_file:=<path_calibration_file>
```

A node must be launched per camera. Multiple nodes can be launched at the same time with little lattency impact. Just calibrate every camera, detect its port and assign it an upcam_id. 

### Other nodes
DonkieTown was released with even more nodes, however, they will be documented in the future. If you are experienced with 


## LAN configuration.
In CIMAT-ZAC we use a router with a wireless local area network to avoid changing network configuration and ros environment variables for each computer whenever we need to relocate. 

- SSID:
   > CIMAT_AUTOMINY
- Password:
   > CIMATZACATECAS
- Gateway:
   > 192.168.100.1

### Agents' static ips.
- Base Station and ROS Master:
   > 192.168.100.100
- Upper cameras:
   > 192.168.100.[100+ID]
- Asinus cars:
   > 192.168.100.[100+MARKER_ID]

All IDs are greater than 0.

Please properly edit $DT_IP with the right ID in DonkiwTown's setup.bash file:
```shell
export DT_IP=[ID]
```
And don't forget to source that file.

### Agents' SSH credentials.
- user_name:
   > donkietown
- password:
   > elfmeter

### Maintance
@L4rralde or Emmanuel Larralde-Ortiz is the responsible to assure the quality of the whole software stack. Don't hesitate to reach him for any kind of concern, support, collaboration or so.

### Contact
> Emmanuel Larralde-Ortiz
- ealarralde@gmail.com
> Alberto Luviano-Juárez
- aluvianoj@ipn.mx
> Diego Mercado-Ravell
- diego.mercado@cimat.mx
> Flabio Mirelez-Delgado
- fmirelezd@ipn.mx