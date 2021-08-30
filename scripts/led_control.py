#!/usr/bin/env python
# -*- coding: utf-8 -*-

#-------------- led_control.py --------------------------------------------------
# Arthor: Arron Griffiths | Company: Indro Robotics | Location: Ottawa, ON, Canada
# Contact: agriffiths@indrorobotics.com  
# Info: This is a simple python ros node that connects to a serial port and pushes 
#       out a string of information define by the numato Lab 8 channel relay 
#       controller.
#       This ros node has been configured for the indro robotics "night rider" 
#       platform, to allow control of the leds via ros service calls
#------------------------------------------------------------------------------

import rospy
import serial
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import ByteMultiArray
from night_rider.msg import LedCmd


class led_control:
#-------------- function deleceration -----------------------------------------
#=======================================================================
    def control_relay_write_read(self, req, relay):
        if(req.data):
            cmd = str("on")
        else:
            cmd = str("off")
        self.serial_port.write("relay " + cmd + " " + str(relay) + "\n\r")
        self.serial_port.flush()
        self.serial_port.write("relay read " + str(relay) + "\n\r")

        response = self.serial_port.read(25)
        if ((response.find("on") > 0) and (req.data)):
            state = True
            response = "Turn on relay " + str(relay)

        elif ((response.find("off") > 0) and (req.data == False)):
            state = True
            response = "Turn off relay " + str(relay)
        else:
            state = False
            response = "!! ERROR !! not able to control relay"
        return SetBoolResponse(state, response)

#=======================================================================
    def control_relay_write(self, req, relay):
        if(req.data):
            cmd = str("on")
        else:
            cmd = str("off")
        self.serial_port.write("relay " + cmd + " " + str(relay) + "\n\r")
        self.serial_port.flush()
        return

#=======================================================================
    def control_relay_toggle(self, toggle, relay):
        if toggle :
            cmd = str("on")
        else :
            cmd = str("off")
        self.serial_port.write("relay " + cmd + " " + str(relay) + "\n\r")
        self.serial_port.flush()
            
        return    

#=======================================================================
    def control_led_0(self, req):
        return self.control_relay_write_read(req, 0)

    def control_led_1(self, req):
        return self.control_relay_write_read(req, 1)

    def control_led_2(self, req):
        return self.control_relay_write_read(req, 2)

    def control_led_3(self, req):
        return self.control_relay_write_read(req, 3)

    def control_led_4(self, req):
        return self.control_relay_write_read(req, 4)

    def control_led_5(self, req):
        return self.control_relay_write_read(req, 5)

    def control_led_6(self, req):
        return self.control_relay_write_read(req, 6)

    def control_led_7(self, req):
        return self.control_relay_write_read(req, 7)

#=======================================================================
    def control_all_leds(self, req):
        self.control_relay_write(req, 0)
        self.control_relay_write(req, 1)
        self.control_relay_write(req, 2)
        self.control_relay_write(req, 3)
        self.control_relay_write(req, 4)
        self.control_relay_write(req, 5)
        self.control_relay_write(req, 6)
        self.control_relay_write(req, 7)
        response = "All LEDs set to: " + str(req.data)
        return SetBoolResponse(True, response)

#=======================================================================
    def control_right_yellow_leds(self, req):
        self.control_relay_write(req, 3)
        self.control_relay_write(req, 5)
        response = "Right Yellow LEDs set to: " + str(req.data)
        return SetBoolResponse(True, response)

#=======================================================================
    def control_left_yellow_leds(self, req):
        self.control_relay_writ(req, 2)
        self.control_relay_write(req, 4)
        response = "Left Yellow LEDs set to: " + str(req.data)
        return SetBoolResponse(True, response)

#=======================================================================
    def callbackSetLEDs(self, msg): 

        self.control_relay_toggle(msg.front_spot_leds, 6)
        print(msg.front_spot_leds)

            
        if msg.rear_red_spot_led :   
            self.control_relay_toggle(1, 0)
        else :
            self.control_relay_toggle(0, 0)
            
       # self.control_relay_write(req.data=msg.front_right_led, 5)
       # self.control_relay_write(req.data=msg.front_left_led, 4)
       # self.control_relay_write(req.data=msg.rear_spot_leds, 1)
       # self.control_relay_write(req.data=msg.rear_left_led, 2)
       # self.control_relay_write(req.data=msg.rear_right_led, 3)
       # self.control_relay_write(req.data=msg.rear_red_spot_led, 0)

#=======================================================================
    def main_loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            rate.sleep()
        self.serial_port.close()

#-----------------------------------------------------------------------

#=======================================================================
    def __init__(self):
#-------------- Serial setup --------------------------------------------------
        self.port = rospy.get_param('~port', '/dev/ttyACM3')
        self.baud = rospy.get_param('~baud', 19200)
        self.serial_port = serial.Serial(self.port, self.baud, timeout=1)
#-------------- function deleceration -----------------------------------------
        rospy.Service('set_rear_red_led', SetBool, self.control_led_0)
        rospy.Service('set_rear_spot_leds', SetBool, self.control_led_1)
        rospy.Service('set_rear_left_yellow_led', SetBool, self.control_led_2)
        rospy.Service('set_rear_right_yellow_led', SetBool, self.control_led_3)       
        rospy.Service('set_front_left_yellow_led', SetBool, self.control_led_4)
        rospy.Service('set_front_right_yellow_led', SetBool, self.control_led_5)
        rospy.Service('set_front_spot_leds', SetBool, self.control_led_6)
        rospy.Service('set_spare_led', SetBool, self.control_led_7)
        rospy.Service('set_all_leds', SetBool, self.control_all_leds)
        rospy.Service('set_right_yellow_leds', SetBool, self.control_right_yellow_leds)
        rospy.Service('set_left_yellow_leds', SetBool, self.control_left_yellow_leds)
        
        rospy.Subscriber("/led_light_control", LedCmd, self.callbackSetLEDs)
        
        self.main_loop()
#-----------------------------------------------------------------------

if __name__ == "__main__":
    rospy.init_node('Led_control_server')
    nri = led_control()
