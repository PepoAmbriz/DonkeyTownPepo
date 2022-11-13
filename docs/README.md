# DonkieTown 0.1
## A low-cost experimental platform for research on Intelligent Vehicles. 

DonkieTown consists of one or more differential-drive robots called Asinus cars, a base station, a localization system and a series of trusted techniques that easily allow the development of testbeds to implement and validate different strategies for collaborative autonomous driving, and study a variety of cases of study.

![DonkieTown status](/docs/images/DonkieTown.jpg)

## Fast configuration.
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
```
export DT_IP=[ID]
```
And don't forget to source that file.


### Agents SSH credentials.
- user_name:
   > donkietown
- password:
   > elfmeter

# Simulator
Multiple simulation scenarios are provided in the bring_up package.
You can launch one scenario, e.g. in-house navigation, with the following command.
```
roslaunch bring_up navigation_inhouse.launch
```
![Simulator](/docs/images/driving_sim.jpg)

# Asinus Cars
Please refer to the jetson_nano branch.
![Asinus Car](/docs/images/AsinusCar.jpg)

# Upper cameras
Please refer to the upper_cam branch.

# Troubleshooting
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
 - [] Create launch files.
 - [] Get actual relative poses of Asinus Cars' cameras.
 - [] Enable individual launch of obstacle detection.