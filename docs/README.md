<div align="center">
<img src="https://github.com/L4rralde/DonkieTown/blob/main/docs/images/AsinusCar.jpg" width="300"/>

# DonkieTown: a Low-cost Experimental Testbed for Research on Autonomous Cars
</div>

### Abstract
*DonkieTown* is an affordable and scalable platform for research on autonomous vehicles. The experimental framework was developed in [ROS](https://www.ros.org/). The platform integrates multiple small scale autonomous vehicles called *Asinus Cars*, which are equipped with at least a camera, odometer, and onboard computer. The vehicles are Differential Drive Robots (DDR), forced by software to behave as car-like vehicles. DonkieTown incorporates a low-cost localization system to provide the real-time vehicles' pose, by means of external cameras which detect ArUco markers, then Kalman Filters (KF) are used to track and estimate the pose of each vehicle. The platform includes a base station computer with a graphical interface for monitoring the system. DonkieTown also includes a series of algorithms to facilitate autonomous driving, such as communication, tracking, object detection, obstacle avoidance, control, trajectory tracking, etc. Moreover, a centralized vehicular network is implemented to allow communication between the agents and the base station, where the agents can share information about their state, obstacles, maneuver intentions, etc.

<div align="center">
<img src="https://github.com/L4rralde/DonkieTown/blob/main/docs/gifs/platoon.gif" width="600"/>
</div>

### [Future] Cite and contribute
All developed source code, libraries and manufacturing files are released as open source under no license agreements. We expect every DonkieTown user to attribute our effort by citing *DonkieTown*.

```bibtex
@inproceedings{

}
```

To contribute, you are free to create, manage and maintain side branches. By the moment direct git pushes to main branch and any forced git push are not allowed, however, you may submit GitHub pull requests and the maintenance team[^1] will review them and decide whether to merge it or not.
[^1]: [@L4rralde](https://github.com/L4rralde) or Emmanuel Larralde-Ortiz is the responsible to assure the quality of the whole software stack. Don't hesitate to reach him for any kind of concern, support, collaboration or so.

&nbsp;


## Installation & Set-up
If you have not installed and set up DonkieTown, visit [the installation page](/docs/INSTALL.md) for directions. You must get it done before using DonkieTown. Once it is done, you may be able to start launching Simulator nodes, Navigation nodes, Localization Nodes, and Vehicular Communication Nodes. 

&nbsp;


## Getting started.
### Primary ROS Nodes and ROS Launches
ROS Nodes are processes capable of subscribing (or listening) and publishing (or talking) to ROS topics while ROS topics are named buses for the broadcasted data. ROS launch is a command that launches one or more nodes with both preset configuration and command-line arguments. The following ROS launches are sufficient to use all DonkieTown functions. 

### ROS nodes and ROS launches

#### Simulator [Base station only]
Multiple simulation scenarios are provided in the bring_up package.
You can launch one scenario, e.g. in-house navigation, with the following command.
```shell
roslaunch bring_up navigation_inhouse.launch
```

<div align="center">
<img src="https://github.com/L4rralde/DonkieTown/blob/main/docs/gifs/simulation.gif" width="600"/>
</div>

#### Asinus Car's *Core* node
Core node is the essential node of the Asinus Cars. This node enables ROS topics to enable the motors, odometry and an Extended Kalman Filter. If you only want to control the Asinus Car, this node is the go. Also, alongside fake_gps node(s), you get an estimation of the robot's absolute position within the road.
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

#### Navigation (standalone) node [Asinus car or Base Station]
It is recommended to use the base station as navigation controller since you could start it and finish it without login to the Asinus Car.
```shell
roslaunch fub_navigation navigation.launch car_id:=<marker_id>
```

#### Indoors localization system node [Base Station or upper cameras]
For this, you should calibrate each camera and recognize the port it is connected to. To calibrate a camera, we recommend the [OpenCV's tutorial](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html).

To detect camera ports, install and use v4l2 utils:
```shell
sudo apt-get install v4l-utils
v4l2-ctl --list-devices
```

Once you have calibrated a camera and detected its port, launch the node.
```shell
roslaunch fake_gps fake_gps.launch upcam_id:=<upcam_id> cam_port:=<cam_port> calib_file:=<path_calibration_file>
```

A node must be launched per camera. Multiple nodes can be launched at the same time with little latency impact. Just calibrate every camera, detect its port, and assign it an upcam_id and pass its calibration file. 

&nbsp;


## Workshops
In Q1 2023, the CIMAT Zacatecas community has provided 3 workshops. Undergraduate students, graduate students, teachers, and entrepreneurs from around Mexico have taken the workshop in Zacatecas (city).

<div align="center">
<img src="https://github.com/L4rralde/DonkieTown/blob/main/docs/images/workshop_poster.png" width="800"/>
</div>

&nbsp;


## Stay tuned
Occasionally visit this repo, documentation is constantly in change and great news will come :wink:. Follow [Tsanda Labs](https://www.youtube.com/@tsandalabs9057/) on YouTube or at least watch its videos. Videos are being producing in this moment. Also, visit this [website](https://mcr.cimat.mx/es/TallerROS) if you want to enroll to the next workshop.

&nbsp;


## Contact
- Emmanuel Larralde-Ortiz
   - ealarralde@gmail.com
- Alberto Luviano-Juárez
   - aluvianoj@ipn.mx
- Diego Mercado-Ravell
   - diego.mercado@cimat.mx
- Flabio Mirelez-Delgado
   - fmirelezd@ipn.mx