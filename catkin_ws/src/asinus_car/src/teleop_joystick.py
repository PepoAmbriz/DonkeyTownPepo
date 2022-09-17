#!/usr/bin/env python 
from __future__ import print_function

import roslib
import rospy
from sensor_msgs.msg import Joy
from donkietown_msgs.msg import MotorsSpeed

class JoyMotor():
    def __init__(self, rate, car_id):
        self.publisher = rospy.Publisher('/asinus_cars/'+str(car_id)+'/motors_driver', MotorsSpeed, queue_size = 1)
        self.speedL = 0.0
        self.speedR = 0.0
        rospy.Subscriber("joy", Joy, self.joy_callback)
        rospy.on_shutdown(self.shutdown)
        self.shutdown_ = False
    def publish(self,speedL,speedR):
        speed_msg = MotorsSpeed()
        speed_msg.leftMotor = self.speedL
        speed_msg.rightMotor = self.speedR
        self.publisher.publish(speed_msg)
    def stop(self):
        self.publish(0.0,0.0)
    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")
    def joy_callback(self,joy_data):
        speedL = 50*joy_data.axes[1]
        speedR = 50*joy_data.axes[0]
        if not self.shutdown_:
            self.publish(speedL,speedR)
    def shutdown(self):
        print("shutdown!")
        self.shutdown_ = True
        self.publish(0.0,0.0)
        rospy.sleep(1)


if __name__=="__main__":
    rospy.init_node('teleop_joystick')
    repeat = rospy.get_param("~repeat_rate", 10.0)
    car_id = rospy.get_param("~car_id", 3)
    motors_node = JoyMotor(repeat,car_id)
    motors_node.wait_for_subscribers()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")