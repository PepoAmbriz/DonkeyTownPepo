#!/usr/bin/env python 
from __future__ import print_function

import roslib
import rospy
from sensor_msgs.msg import Joy
from donkietown_msgs.msg import MotorsSpeed
import pygame
from math import tanh
BLACK = pygame.Color('black')
WHITE = pygame.Color('white')

class TextPrint(object):
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 30)

    def tprint(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, (self.x, self.y))
        self.y += self.line_height

    def reset(self):
        self.x = 15
        self.y = 15
        self.line_height = 25

    def indent(self):
        self.x += 15

    def unindent(self):
        self.x -= 15

class JoyMotor():
    def __init__(self, rate, car_id,r=0.03,L=0.1):
        self.topic_name = '/asinus_cars/'+str(car_id)+'/motors_driver'
        self.publisher = rospy.Publisher(self.topic_name, MotorsSpeed, queue_size = 1)
        self.stop()
        rospy.on_shutdown(self.shutdown)
        self.shutdown_ = False
    def publish(self,speedL,speedR):
        speed_msg = MotorsSpeed()
        speed_msg.leftMotor = speedL
        speed_msg.rightMotor = speedR
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
    def shutdown(self):
        print("shutdown!")
        self.shutdown_ = True
        self.publish(0.0,0.0)
        rospy.sleep(1)

    def talk(self,rate):
        pygame.init()
        screen = pygame.display.set_mode((500, 200))
        clock = pygame.time.Clock()
        pygame.joystick.init()
        textPrint = TextPrint()
        while not self.shutdown_:
            for event in pygame.event.get(): # User did something.
                if event.type == pygame.QUIT: # If user clicked close.
                    self.shutdown_ = True # Flag that we are done so we exit this loop.
            screen.fill(WHITE)
            textPrint.reset()
            try:
                joystick = pygame.joystick.Joystick(0)
            except:
                pygame.display.flip()
                clock.tick(rate)
                continue
            textPrint.tprint(screen, "Publishing to topic {}".format(self.topic_name))
            textPrint.indent()
            textPrint.indent()
            joystick.init()
            # Usually axis run in pairs, up/down for one, and left/right for
            # the other.
            axes = joystick.get_numaxes()                
            if axes==0:
                continue
            s_kind = int(-60*tanh(joystick.get_axis(1)))
            w_kind = int(-30*tanh(joystick.get_axis(2)))   
            speedL = s_kind-0.5*w_kind
            speedR = s_kind+0.5*w_kind

            self.publish(speedL,speedR)
            textPrint.tprint(screen, "{} value: {:>6.3f}".format("Axis_s", s_kind))
            textPrint.tprint(screen, "{} value: {:>6.3f}".format("Axis_w", w_kind))
            textPrint.tprint(screen, "{} value: {:>6.3f}".format("speedL", speedL))
            textPrint.tprint(screen, "{} value: {:>6.3f}".format("speedR", speedR))

            pygame.display.flip()
            clock.tick(rate)
        pygame.quit()




if __name__=="__main__":
    rospy.init_node('teleop_joystick')
    repeat = rospy.get_param("~repeat_rate", 10.0)
    car_id = rospy.get_param("~car_id", 3)
    motors_node = JoyMotor(repeat,car_id,r=0.03,L=0.1)
    #motors_node.wait_for_subscribers()
    try:
        motors_node.talk(5)
    except KeyboardInterrupt:
        print("Shutting down")