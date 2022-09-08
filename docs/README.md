# DonkieTown (WIP)
## A low-cost experimental platform for research on Intelligent Vehicles. 

DonkieTown consists of one or more differential-drive robots called Asinus cars, a ground station, a localization system and a series of trusted techniques that easily allow the development of testbeds to implement and validate different strategies for collaborative autonomous driving, and study a variety of cases of study.

## Fast configuration.
In CIMAT-ZAC we use a router with a wireless local area network to avoid changing network configuration and ros environment variables for each computer whenever we need to relocate. 

- SSID:
   > CIMAT_AUTOMINY
- Password:
   > CIMATZACATECAS
- Gateway:
   > 192.168.100.1

### Agents static ips.
- Ground Station and ROS Master:
   > 192.168.100.100
- Upper cameras:
   > 192.168.100.[100+ID]
- Asinus cars:
   > 192.168.100.[100+MARKER_ID]

All IDs are greater than 0.

### Agents SSH credentials.
- user_name:
   > donkietown
- password:
   > elfmeter

## TODO
### Enabling fast wlan access configuration.
 - [x] Document wlan CIMAT_AUTOMINY. 
 - [ ] (Troubleshooting) Document how to enable netplan with wlan0 static ip. 
 - [ ] Set all ips to static.
 - [ ] (Troubleshooting) Document how to modify 
 - [ ] Change all user_names and pswds to donkietown and elfmeter.

### Troubleshooting
#### Setting static ip only for wlan
This procedure has been proved for asinus cars and ours upper cameras. 
1. Install netplan.
```
sudo apt-get install netplan
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
