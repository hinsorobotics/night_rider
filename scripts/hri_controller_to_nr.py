#!/usr/bin/env python
# -*- coding: utf-8 -*-

#-------------- hri_controller_remap.py -----------------------------------------------------------
# Arthor: Arron Griffiths | Company: Indro Robotics | Location: Ottawa, ON, Canada
# Contact: agriffiths@indrorobotics.com  
# Info: This is a simple python ros node that subs to the published topics on "/joy" and 
#       "/hrI_src/emergency_stop" that are controller by the HRI (FORT Robotics) Wireless
#       controller (SRC)
#       This ros node has been configured for the indro robotics "night rider" platform,
#       to allow control of the leds via ros service calls and the "/cmd_val" topic
#       via publishing new "/cmd_val" vales, which is published to the night rider ros driver (nr_ctrl_cmd)
#---------------------------------------------------------------------------------------------------

import rospy
from std_msgs.msg import ByteMultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from night_rider.msg import LedCmd

class hri_controller_to_nr: 
#=======================================================================
	def callbackJoy2twist(self, data):
#------ declearations
		twist = Twist()
		
		
		# ----- Braking -----
		# hri publishes on the "joy" topic where the left trigger 
		# is used as a "death man" / break safety input. Thispull down and 
		# keep this trigger down to dis-enage the break. If the use is not
		# focused or is kocked or worse, then this trigger will reset and the 
		# break will be enaged for safety means the user
		twist.linear.z = 1 - data.buttons[10]
		if twist.linear.z == 1 :
			self.led_cmd.rear_red_spot_led = 1
		else :
			self.led_cmd.rear_red_spot_led = 0
			
		# ----- Speed and Gear -----
		# hri publishes on the "joy" topic where the left analog stick
		# is use to deturmian the speed of the NR and
		# the direction of travel (forward or backward)
		# "joy" is normiziled to -1~0~1, this can be scale to the Maxium 
		# m/s of the NR, which is approx 5m/s with it curent gearing
		# Drive and reserves is deturmained by the postive/negitive of the joy value
		# if joy value = 0 the NR will be in neutarl and roll (if brake not engaged)  
		twist.linear.x = data.axes[1] * self.scaler_nr_joy2cmd_linear_x
		
		# ----- Steer -----
		# hri publishes on the "joy" topic where the right analog stick
		# is use to deturmian the steer of the NR and
		# the direction of travel via turning (right or left)
		# "joy" is normiziled to -1~0~1, for NR platform the maxium
		# angle of steering is 28 deg. (nr_ctrl_cmd.py has the scaler) 
		# Inverted "data.axes" as the pos and neg numbers are oppersite in ROS TF
		twist.angular.z = -data.axes[2] 
		
		
		if data.buttons[6] == 1 and self.last_button_state == 0 :
			self.led_cmd.front_spot_leds = not self.led_cmd.front_spot_leds 
			print (self.led_cmd.front_spot_leds)

		#if data.buttons[4] == 1 and self.last_button_state[4] == 0 :
		#	if self.rear_spot_leds == 0 :
		#		self.led_cmd.rear_spot_leds = 1
		#		self.front_spot_leds = 1
		#	elif self.front_spot_leds == 1 :
		#		self.led_cmd.front_spot_leds = 0
		#		self.front_spot_leds = 0
			
		
		self.last_button_state = data.buttons[6]
		
		self.pubLEDs.publish(self.led_cmd)
		
		self.pubTwist.publish(twist)
#-----------------------------------------------------------------------  
		
#=======================================================================
	def main_loop(self) :
		rospy.loginfo("HRI Joy To Night Rider Node Started")

		while not rospy.is_shutdown(): 
			
			rospy.spin()
#-----------------------------------------------------------------------  
			
#=======================================================================
	def __init__(self):
		# publishing to "/cmd_vel" to control night rider
		global pubTwist, pubLEDs
		global led_cmd, led_cmd_pervious, front_spot_leds, last_button_state
		
		self.front_spot_leds = bool()
		self.last_button_state = bool()
		
		self.front_spot_leds = 0
		
		self.led_cmd = LedCmd()
		self.led_cmd_pervious = LedCmd()
		
		
		rospy.init_node('hri_controller_to_nr')
		
		self.pubTwist = rospy.Publisher('/night_rider/cmd_vel', Twist, queue_size=10)
		self.pubLEDs = rospy.Publisher('/led_light_control', LedCmd, queue_size=1)
		
		# subscribed to joystick inputs on topic "joy"
		rospy.Subscriber("joy", Joy, self.callbackJoy2twist)
		
		self.scaler_nr_joy2cmd_linear_x = rospy.get_param('~joy2cmd_linear_x', 4)
		self.scaler_nr_joy2cmd_node_hz = rospy.get_param('~joy2cmd_node_hz', 20)
		rospy.Rate(self.scaler_nr_joy2cmd_node_hz)
		
		# starts the node
		self.main_loop()
#-----------------------------------------------------------------------  

if __name__ == '__main__':
    hri_controller_to_nr()
