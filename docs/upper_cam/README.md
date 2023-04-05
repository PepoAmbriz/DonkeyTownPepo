# DonkieTown
## A low-cost experimental platform for research on Intelligent Vehicles. 

DonkieTown consists of one or more differential-drive robots called Asinus cars, a ground station, a localization system and a series of trusted techniques that easily allow the development of testbeds to implement and validate different strategies for collaborative autonomous driving, and study a variety of cases of study.

## Common installation instructions
### Setting static ip only for wlan
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

## Modules installation
### cv_bridge
```
sudo apt-get install ros-(ROS version name)-vision-opencv
sudo apt-get install ros-(ROS version name)-cv-bridge
```
### requests python module
```
sudo apt install python-pip #If not installed
pip install requests
```
### tf ros
```
sudo apt install ros-melodic-tf
```