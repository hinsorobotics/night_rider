#!/usr/bin/env python
# -*- coding: utf-8 -*-

#-------------- hri_controller_remap.py ------------------------------------------------
# Arthor: Arron Griffiths | Company: Indro Robotics | Location: Ottawa, ON, Canada
# Contact: agriffiths@indrorobotics.com  
# Info: This is a simple python ros node that subs to the published topics on "/joy" and 
#       "/hrI_src/emergency_stop" that are controller by the HRI (FORT Robotics) Wireless
#       controller (SRC)
#       This ros node has been configured for the indro robotics "night rider" platform,
#       to allow control of the leds via ros service calls and the "/cmd_val" topic
#       via publishing new "/cmd_val" vales, which is published ny the wego erp4 ros driver
#---------------------------------------------------------------------------------------------------

import rospy
from std_msgs.msg import ByteMultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def callbackJoy2twist(data):
    twist = Twist()
    twist.linear.x = data.axes[1]
    twist.angular.z = data.axes[2]
    ledArray = ByteMultiArray()
    
    if data.buttons[6] == 1:
	    ledArray.data[0] = not ledArray.data[0]
    
    pubLEDs.publish(ledArray)
    pubTwist.publish(twist)
    
    
    

# Intializes everything
def start():
    # publishing to "/cmd_vel" to control night rider
    global pubTwist, pubLEDs
    
    pubTwist = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pubLEDs = rospy.Publisher('/led_light_control', ByteMultiArray, queue_size=1)
    
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callbackJoy2twist)
    
    # starts the node
    rospy.init_node('hri_Joy_to_night_rider')
    rospy.spin()

if __name__ == '__main__':
    start()
