# DonkieTown
## A low-cost experimental platform for research on Intelligent Vehicles. 

DonkieTown consists of one or more differential-drive robots called Asinus cars, a ground station, a localization system and a series of trusted techniques that easily allow the development of testbeds to implement and validate different strategies for collaborative autonomous driving, and study a variety of cases of study.

## Fast configuration.
In CIMAT-ZAC we use a router with a wireless local area network to avoid changing network configuration and ros environment variables for each computer whenever we need to relocate. 

SSID:
> CIMAT_AUTOMINY

Password:
> CIMATZACATECAS

Gateway:
> 192.168.100.1

### Agents static ips.
Ground Station and ROS Master:
> 192.168.100.100

Upper cameras:
> 192.168.100.[100+ID]

Asinus cars:
> 192.168.100.[100+CAR_ID]

All IDs are greater than 0.

### Agents SSH credentials.
user_name:
> donkietown
password:
> elfmeter

## TODO
### Enabling fast wlan access configuration.
 - [ ] Document wlan CIMAT_AUTOMINY. 
 - [ ] (Troubleshooting) Document how to enable netplan with wlan0 static ip. 
 - [ ] Set all ips to static.
 - [ ] (Troubleshooting) Document how to modify 
 - [ ] Change all user_names and pswds to donkietown and elfmeter.

### Troubleshooting