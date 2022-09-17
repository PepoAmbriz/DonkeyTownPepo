#!/usr/bin/python 
from __future__ import print_function

import threading

import roslib
import rospy

from donkietown_msgs.msg import MotorsSpeed

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to motors driver
w/s : increase/decrease left motor speed by 10%
o/l : increase/decrease right motor speed by 10%
anything else : stop
CTRL-C to quit
"""

speedBindings={
        'w':(5.0,0.0),
        's':(-5.0,0.0),
        'o':(0.0,5.0),
        'l':(0.0,-5.0),
    }

class PublishThread(threading.Thread):
    def __init__(self, rate, car_id):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('/asinus_cars/'+str(car_id)+'/motors_driver', MotorsSpeed, queue_size = 1)
        self.speedL = 0.0
        self.speedR = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

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

    def update(self, speedL, speedR):
        self.condition.acquire()
        self.speedL = speedL
        self.speedR = speedR
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0.0, 0.0)
        self.join()

    def run(self):
        speed_msg = MotorsSpeed()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            speed_msg.leftMotor = self.speedL
            speed_msg.rightMotor = self.speedR
            """
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn
			"""
            self.condition.release()

            # Publish.
            self.publisher.publish(speed_msg)

        # Publish stop message when thread exits.
        speed_msg.leftMotor = 0.0
        speed_msg.rightMotor = 0.0
        self.publisher.publish(speed_msg)


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speedL, speedR):
    return "currently:\tspeedL %s\tspeedR %s " % (speedL,speedR)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_keyboard')

    speedL = rospy.get_param("~speedL", 10.0)
    speedR = rospy.get_param("~turn", 10.0)
    repeat = rospy.get_param("~repeat_rate", 10.0)
    car_id = rospy.get_param("~car_id", 3)
    key_timeout = rospy.get_param("~key_timeout", 2.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat,car_id)

    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(speedL, speedR)

        print(msg)
        print(vels(speedL,speedR))
        while(1):
            key = getKey(key_timeout)
            if key in speedBindings.keys():
                speedL = speedL + speedBindings[key][0]
                speedR = speedR + speedBindings[key][1]

                print(vels(speedL,speedR))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '':
                    continue
                if (key == '\x03'):
                    break
 
            pub_thread.update(speedL, speedR)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)