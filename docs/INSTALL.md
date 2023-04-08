# Installing **DonkieTown** software
DonkieTown is a platform which consists of one or more differential-drive robots called Asinus cars, a base station, a localization system, and a series of trusted techniques that easily allow the development of testbeds to implement and validate different strategies for collaborative autonomous driving and study a variety of cases of study. [More](/docs/README.md).

&nbsp;


## Asinus Cars
For instructions to install Asinus Cars' software refer to [the jetson_nano page](/docs/jetson_nano/README.md).

&nbsp;


## Upper cameras
For instructions for the Upper Cameras (if you want to use other computers in addition to the Base Station) please refer to [the upper_cam page](/docs/upper_cam/README.md). If you are using the base station to process upper camera's video, you are good with the instructions for the Base Station.

&nbsp;


## Base Station
DonkieTown was developed in ROS Melodic Morenia, however, we have evaluated some ROS packages in ROS Noetic, the last supported version of ROS1. You could try it over ROS Noetic if you are experienced with ROS. Anyway, the following steps are the recommended flow:

1. Install Ubuntu 18.04 LTS Desktop version from the [official release](https://releases.ubuntu.com/18.04/).

2. Install ROS Melodic Morenia Desktop version as described in [*Ubuntu install of ROS Melodic*](http://wiki.ros.org/melodic/Installation/Ubuntu)

3. Update your sources.
```shell
sudo apt update
```

3. Install the following ROS libraries:
```shell
sudo apt update
sudo apt install ros-melodic-tf
sudo apt-get install ros-melodic-cv-bridge
sudo apt-get install ros-melodic-vision-opencv
```

4. Install the following python modules:
```shell
sudo apt-get install python-pip
pip install -U pygame
pip install numpy
python -m pip install scipy
python -m pip install -U matplotlib
```

5. Install git.
```shell
sudo apt install git
```

6. Clone and build DonkieTown.
```shell
cd ~
git clone https://github.com/L4rralde/DonkieTown.git .
```

7. Build DonkieTown.
```shell
cd ~/DonkieTown/ros_base_nodes/catkin_ws
catkin_make
cd ~/DonkieTown/ros_station_nodes/catkin_ws
catkin_make
```

8. Source DonkieTown.
```shell
source ~/DonkieTown/bs_setup.bash
```
Permanently,
```shell
sudo sh -c "echo 'source ~/DonkieTown/bs_setup.bash' >> ~/.bashrc"
```

For any issue contact the maintenance team[^1].
[^1]: [@L4rralde](https://github.com/L4rralde) or Emmanuel Larralde-Ortiz is the responsible to assure the quality of the whole software stack. Don't hesitate to reach him for any kind of concern, support, collaboration or so.

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