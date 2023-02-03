# DonkieTown
## A low-cost experimental platform for research on Intelligent Vehicles. 

DonkieTown consists of one or more differential-drive robots called Asinus cars, a ground station, a localization system and a series of trusted techniques that easily allow the development of testbeds to implement and validate different strategies for collaborative autonomous driving, and study a variety of cases of study.

## Instructions
- Flash Jetpack 4.5 on 64GB pendrive with balena etcher or flash jetbot 0.4.3 image as described in https://jetbot.org/master/software_setup/sd_card.html

- Install ros melodic and ros packages for jetson_nano as described in https://github.com/dusty-nv/jetbot_ros/tree/melodic 

- Proceed with official instructions to install "Dependencies for building packages":

Install rosinstall and dependencies:
```
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```
Install rosdep
```
sudo apt install python-rosdep
```
Initialize rosdep
```
sudo rosdep init
rosdep update
```

- Install ros_deep_learning nodes:
install ros dependencies for ros_deep_learning nodes:
```
sudo apt-get install ros-melodic-image-transport ros-melodic-vision-msgs
```
Create ros workspace dir for ros_deep_learning
```
mkdir -p ~/ros_deep_learning/catkin_ws/src
cd ~/ros_deep_learning/catkin_ws/
catkin_make
```
Clone forked version:
```
cd ~/ros_deep_learning/catkin_ws/src
git clone https://github.com/L4rralde/ros_deep_learning.git
```
Build ros_deep_learning package
```
cd ~/ros_deep_learning/catkin_ws/
catkin_make
```
Source package
```
sudo sh -c "echo 'source ~/ros_deep_learning/catkin_ws/devel/setup.bash' >> ~/.bashrc"
```

- Build pylibi2c from: https://github.com/amaork/libi2c

- [OPTIONAL] Build source of openCV 4.5.0 for jetson nano with cuda enabled as described in: https://qengineering.eu/install-opencv-4.5-on-jetson-nano.html

- [OPTIONAL] Install scipy for python2:
```
python -m pip install --upgrade pip
python -m pip install scipy
```
or 
```
sudo apt-get install python-scipy
```
- [DEPRECATED] Install filterpy with pip
```
pip install filterpy
```
- [DEPRECATED] Install filterpy from source
```
git clone http://github.com/rlabbe/filterpy
python setup.py install
```

- Install the DonkieTown repo:
First, install ros packages dependencies:
```
sudo apt-get install ros-melodic-cv-bridge ros-melodic-tf
```

```
git clone https://github.com/L4rralde/DonkieTown.git
cd <DT_PATH>/ros_base_nodes/catkin_ws
catkin_make
```

- Set Asinus Car's id. 
The Asinus Car's id is the same as the id of the attached aruco marker. 
```
nano <DT_PATH>/ag_setup.bash
```
There
```
DT_ID=<Asinus Car's id>
```
Source ag_setup.bash file
```
sudo sh -c "echo 'source <DT_PATH>/ag_setup.bash' >> ~/.bashrc"
```

## Troubleshooting
- Project 'cv_bridge' can not find opencv

In /opt/ros/melodic/share/cv_bridge/cmake/cv_bridgeConfig.cmake, replace this line:
```
set(_include_dirs "include;/usr/include;/usr/include/opencv")
```
to
```
set(_include_dirs "include;/usr/include;/usr/include/opencv4")
```

- Temporally, remove lane_following package
```
rm -rf $DT_PATH/ros_base_nodes/catkin_ws/src/lane_following
```