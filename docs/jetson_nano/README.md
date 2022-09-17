# DonkieTown
## A low-cost experimental platform for research on Intelligent Vehicles. 

DonkieTown consists of one or more differential-drive robots called Asinus cars, a ground station, a localization system and a series of trusted techniques that easily allow the development of testbeds to implement and validate different strategies for collaborative autonomous driving, and study a variety of cases of study.

## Instructions
- Flash Jetpack 4.5 on 64GB pendrive with balena etcher or flash jetbot 0.4.3 image as described in https://jetbot.org/master/software_setup/sd_card.html

- Install ros melodic and ros packages for jetson_nano as described in https://github.com/dusty-nv/jetbot_ros/tree/melodic 

- Install scipy:
```
python -m pip install --upgrade pip
python3 -m pip install --upgrade pip
python -m pip install scipy
```
- Install filterpy with pip
```
pip install filterpy
```
- Install filterpy from source
```
git clone http://github.com/rlabbe/filterpy
python setup.py install
```
- Build pylibi2c from: https://github.com/amaork/libi2c

- Build source of openCV 4.5.0 for jetson nano with cuda enabled as described in: https://qengineering.eu/install-opencv-4.5-on-jetson-nano.html


## Status
### Asinus cars
 - First asinus car assembled.
   - [x] Change login credentials.
   - [x] Set static ip.
 - Second in progress:
   - [x] Replace 310-rpm motors with 155-rpm motors.
   - [x] Change login credentials.
   - [x] Set static ip.
 - Third is TODO.
   - [ ] Find h-bride
   - [ ] Solder circuit board.
   - [ ] Build wires.

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