#!/usr/bin/python 
from __future__ import print_function

import threading

import roslib
import rospy

import sys, select, termios, tty
from std_msgs.msg import Int16

msg = """
Reading from the keyboard  and Publishing to motors driver
w/s : increase/decrease left motor speed by 10%
o/l : increase/decrease right motor speed by 10%
anything else : stop
CTRL-C to quit
"""

speedBindings={
        'w':(1.0,0.0),
        's':(-1.0,0.0),
        'o':(0.0,1.0),
        'l':(0.0,-1.0),
    }

class PublishThread(threading.Thread):
    def __init__(self, rate, pubtopic):
        super(PublishThread, self).__init__()
        self.pub_speed = rospy.Publisher(pubtopic+"/manual_control/speed",    Int16, queue_size=1)
        self.pub_steer = rospy.Publisher(pubtopic+"/manual_control/steering", Int16, queue_size=1)
        self.steering = 0.0
        self.speed = 0.0
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
        while not rospy.is_shutdown() and self.pub_speed.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.pub_speed.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, steering, speed):
        self.condition.acquire()
        self.steering = steering
        self.speed = speed
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0.0, 0.0)
        self.join()

    def run(self):
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            self.condition.release()

            # Publish.
            self.pub_steer.publish(self.steering)
            self.pub_speed.publish(self.speed)

        # Publish stop message when thread exits.
        self.pub_steer.publish(0.0)
        self.pub_speed.publish(0.0)

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(steering, speed):
    return "currently:\tsteering %s\tpeed %s " % (steering,speed)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_keyboard')

    repeat = rospy.get_param("~repeat_rate", 10.0)
    key_timeout = rospy.get_param("~key_timeout", 2.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat,"/AutoModelMini")

    status = 0
    steering = 0
    speed = 0
    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(steering+90, -speed)

        print(msg)
        print(vels(steering,speed))
        while(1):
            key = getKey(key_timeout)
            if key in speedBindings.keys():
                steering = steering + 10*speedBindings[key][0]
                speed = speed + 10*speedBindings[key][1]

                print(vels(steering,speed))
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
 
            pub_thread.update(steering+90, -speed)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)