# Installing software for upper cameras.

&nbsp; 


## installation instructions
If you want to enlarge the number of upper cameras and the base station is overheaded. You can add more computers and connect to them cameras. While developing, we use the Raspberry Pi 3B+ Sinble-Board Computer, but Raspberry Pi 4B is preferred. You must install a light version of Ubuntu 18 and ROS Melodic (same instructions as for the base station). Next, follow these directions.

### Update your sources
```shell
sudo apt update
```

### Setting static ip only for wlan
This procedure has been proved for asinus cars and ours upper cameras. 
1. Install netplan.
```shell
sudo apt-get install netplan.io
```
2. Identify your wireless network interface name.
```shell
ls /sys/class/net
```
Take a note of this interface name (wlan0). We will use it in next steps. 

3. Add netplan file from main branch of DonkieTown repo and modify it for the appropriate desired ip.
```shell
sudo cp ./misc/50-cloud-init.yaml /etc/netplan/50-cloud-init.yaml
``` 
4. Generate the configuration.
```shell
sudo netplan generate
```
5. Apply the changes.
```shell
sudo netplan apply
```

### Changing login credentials.
Change the password is as easy as type:
```shell
passwd
```
However, changing username is not as easy (but you could set your crdentials since the first boot).
1. Create a temp user.
```shell
sudo -s
useradd -G sudo temp
passwd temp
shutdown -h now
```
2. Log back in as the new "temp" user and elevate "temp" privileges.
```shell
ssh temp@192.168.100...
sudo -s
```
3. Now, change username of the main account.
```shell
usermod -l donkietown <previous_user_name>
shutdown -h now
```
4. Log back in as the donkietown user and delete out the temp user.
```shell
ssh donkietown@192.168.100.<100+<upcam_id>>
sudo -s
deluser temp
rm -rf /home/temp
```

### Modules installation
#### cv_bridge
```shell
sudo apt-get install ros-melodic-vision-opencv
sudo apt-get install ros-melodic-cv-bridge
```

#### Install python modules
```shell
sudo apt install python-pip 
pip install numpy
pip install requests
```

#### tf ros
```shell
sudo apt install ros-melodic-tf
```

### Clone and build DonkieTown
1. Install git.
```shell
sudo apt install git
```

2. Clone and build DonkieTown.
```shell
cd ~
git clone https://github.com/L4rralde/DonkieTown.git .
```

3. Build DonkieTown's base packages.
```shell
cd ~/DonkieTown/ros_base_nodes/catkin_ws
catkin_make
```

4. Source DonkieTown.
```shell
source ~/DonkieTown/ag_setup.bash
```
Permanently,
```shell
sudo sh -c "echo 'source ~/DonkieTown/ag_setup.bash' >> ~/.bashrc"
```

&nbsp;


## Troubleshooting
If there is any packages, other than fake_gps and donkietown_msgs, blocking `catkin_make`, remove it. For any issue contact the maintenance team[^1].

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