# DonkieTown (Beta)
## A low-cost experimental platform for research on Intelligent Vehicles. 

DonkieTown consists of one or more differential-drive robots called Asinus cars, a base station, a localization system and a series of trusted techniques that easily allow the development of testbeds to implement and validate different strategies for collaborative autonomous driving, and study a variety of cases of study.

[![DonkieTown status](/docs/images/DonkieTown.jpg)](https://youtube.com/playlist?list=PLO9tS78GvAmSHxlJAngOm7qDcSlMZyhZU)

# How to use
## Base Station
It is highly recommended to use a personal computer such as a laptop or workstation as the Base Station. The Base Station plays as ROS Master and Vehicular Network Manager. In order to setup your computer as ros_master and source simulator-only and base-station-only nodes you shall source:
```
source <DT_PATH>/bs_setup.bash
```
Permanently,
```
sudo sh -c "echo 'source <DT_PATH>/bs_setup.bash' >> ~/.bashrc"
```
once it is done, `$DT_PATH` environment variable is set and you may be able to start launching Simulator nodes, Navigation nodes, and Vehicular Communication Nodes.

### Asinus Car's *Core* node
Core node is the essential node of the asinus cars. This node enables ros topics to enable the motors, odometry and an Extended Kalman Filter. If you only want to control the asinus car, this node is the go. Also, alongside fake_gps node(s), you may get an estimation of the robot's absolut position within the road.
```
roslaunch asinus_car core.launch car_id:=<marker_id>
```

### Asinus Car's *Prime* node
In addition to *Core* features, *Prime* enables *DonkieNet* (a MobileNet+SSD detection network trained with a hand-labeled dataset of stuffed white donkeys).
```
roslaunch asinus_car prime.launch car_id:=<marker_id>
```

### Vehicular communication node [Base Station only]
The following starts message handling and post-processing of shared data of pedestrians detected by all Asinus Cars.
```
roslaunch vehicular_communication network.launch
```

### Shared data postprocessing (standalone) node [Base Station only]
```
roslaunch road_context obstacle_localization.launch
```

### Navigation (standalone) node [Asinus car or Base Station]
It is recommended to use the base station for navigation controller since you could start it and finish it without login to the Asinus Car.
```
roslaunch fub_navigation navigation.launch car_id:=<marker_id>
```

## Simulator [Base station only]
Multiple simulation scenarios are provided in the bring_up package.
You can launch one scenario, e.g. in-house navigation, with the following command.
```
roslaunch bring_up navigation_inhouse.launch
```
![Simulator](/docs/images/driving_sim.jpg)

## Asinus Cars
Please refer to [the jetson_nano page](/docs/jetson_nano/README.md)
![Asinus Car](/docs/images/AsinusCar.jpg)

## Upper cameras
Please refer to [the upper_cam page](/docs/upper_cam/README.md).

# LAN configuration.
In CIMAT-ZAC we use a router with a wireless local area network to avoid changing network configuration and ros environment variables for each computer whenever we need to relocate. 

- SSID:
   > CIMAT_AUTOMINY
- Password:
   > CIMATZACATECAS
- Gateway:
   > 192.168.100.1

## Agents' static ips.
- Base Station and ROS Master:
   > 192.168.100.100
- Upper cameras:
   > 192.168.100.[100+ID]
- Asinus cars:
   > 192.168.100.[100+MARKER_ID]

All IDs are greater than 0.

Please properly edit $DT_IP with the right ID in DonkiwTown's setup.bash file:
```
export DT_IP=[ID]
```
And don't forget to source that file.


## Agents' SSH credentials.
- user_name:
   > donkietown
- password:
   > elfmeter

# Contact
For more information you may reach me out.
> Emmanuel Larralde
> ealarralde@gmail.com

# Common installation instructions
## Setting static ip only for wlan
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

# TODO
 - [ ] Label this version.
 - [ ] test cooperative driving node clean-up.
 - [ ] Prepare a demo video and add it.