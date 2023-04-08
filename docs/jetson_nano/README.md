<div align="center">
<img src="https://github.com/L4rralde/DonkieTown/blob/main/docs/images/AsinusCar.jpg" width="400"/>

# Installing software for Asinus Cars 
</div>

&nbsp;


## Instructions
The NVIDIA Jetson Nano Developer Kit is the onboard computer of the Asinus Cars. Therefore, the following instructions apply only for that Single-Board computer.

### 1. FLash Jetpack
- Flash Jetpack 4.5 on 64GB pendrive with balena etcher or flash jetbot 0.4.3 image as described in https://jetbot.org/master/software_setup/sd_card.html

### Setting static ip for wlan
This procedure has been proved for asinus cars and ours upper cameras. 
1. Install netplan.
```
sudo apt-get install netplan.io
```
2. Identify your wireless network interface name.
```
ls /sys/class/net
```
Take a note of this interface name (wlan0). We will use it in next steps. 

3. Add netplan file from main branch of DonkieTown repo.
```
sudo cp ./misc/50-cloud-init.yaml /etc/netplan/50-cloud-init.yaml
``` 
4. Generate the configuration.
```
sudo netplan generate
```
5. Apply the changes.
```
sudo netplan apply
```

### Changing login credentials.
Change the password is as easy as type:
```
passwd
```
However, changing username is not as easy (but you could set your crdentials since the first boot).
1. Create a temp user.
```
sudo -s
useradd -G sudo temp
passwd temp
shutdown -h now
```
2. Log back in as the new "temp" user and elevate "temp" privileges.
```
ssh temp@192.168.100...
sudo -s
```
3. Now, change username of the main account.
```
usermod -l donkietown <previous_user_name>
shutdown -h now
```
4. Log back in as the donkietown user and delete out the temp user.
```
ssh donkietown@192.168.100...
sudo -s
deluser temp
rm -rf /home/temp
```

### 2. Install ROS
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

### 3. Install ROS Deep Learning Nodes
- Install Jetson-Inference as described in https://github.com/dusty-nv/ros_deep_learning#jetson-inference

```
cd ~
sudo apt-get install git cmake
git clone --recursive https://github.com/dusty-nv/jetson-inference
cd jetson-inference
mkdir build
cd build
cmake ../
make -j$(nproc)
sudo make install
sudo ldconfig
```
- install ros dependencies for ros_deep_learning nodes:
```
sudo apt-get install ros-melodic-image-transport ros-melodic-vision-msgs
```
- Create ros workspace dir for ros_deep_learning
```
mkdir -p ~/ros_deep_learning/catkin_ws/src
cd ~/ros_deep_learning/catkin_ws/
catkin_make
```
- Clone forked version:
```
cd ~/ros_deep_learning/catkin_ws/src
git clone https://github.com/L4rralde/ros_deep_learning.git
```
- Build ros_deep_learning package
```
cd ~/ros_deep_learning/catkin_ws/
catkin_make
```
- Source package
```
sudo sh -c "echo 'source ~/ros_deep_learning/catkin_ws/devel/setup.bash' >> ~/.bashrc"
```

### 4. Install DonkieTown Software
- Build pylibi2c from: https://github.com/amaork/libi2c

- [OPTIONAL] Build source of openCV 4.5.0 for jetson nano with cuda enabled as described in: https://qengineering.eu/install-opencv-4.5-on-jetson-nano.html

- Install scipy for python2:
```
python -m pip install --upgrade pip
python -m pip install scipy
```
or 
```
sudo apt-get install python-scipy
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

### 5. Config your Asinus Car
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

&nbsp;


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