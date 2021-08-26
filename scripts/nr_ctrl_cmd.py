#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import struct
import time
from std_msgs.msg import UInt8, Int16
from geometry_msgs.msg import Twist
from night_rider.msg import CtrlCmd, RecvStatus
import rospy
import rospkg
import message_filters

debug = 0
packet = [0 for i in range(7)]  # creat array for packet


class nr_serial_test: 
#=======================================================================
    def cmd_vel_callback(self, cmd_vel) :
        
        if debug :
            rospy.loginfo(" ROS CALL BACK RECIVED ")
            rospy.info(str(cmd_vel))
				
		# ---- NR controller mode ------		
        cmd_mode = 1  		# set mode to 1 = Auto/computer control (O = manual mode for the black controller)
        
        # ---- NR eStop state ------
        cmd_e_stop = 0	    # set to zero at all time by the computer estop is seperate system
        
        # ---- control of NR gears ------
        # To more forward mode need to be "drive"
        # To more backwards mode need to be "reverse"
        # safe state is "neutral" where NR will roll if push or under inertia
        # "cmd_vel.linear.x" is being used for both speed and direction to stay in ROS norms
        if cmd_vel.linear.x > 0.01 :
             cmd_gear = 0 				# Drive
        elif cmd_vel.linear.x < -0.01 :
            cmd_gear = 3				# Reverse
            cmd_vel.linear.x = not cmd_vel.linear.x
        else :
            cmd_gear = 1				# Neutral

		# ---- NR speed  ------
		# The NR drive use gears to define forward or backwards
		# The maxium unit to the NR MCU is 1000 units
		# Maxium speed of current geared NR is 8.2M/s (~30KM)
		# "scaler_nr_speed" is to match the MCU units to ROS twist m/s velosicty 
        cmd_speed = cmd_vel.linear.x * self.scaler_nr_speed 	
        
        # ---- NR steer  ------
        # As NR is Ackermen, cmd_velangular.z this normalized to 1.0 - 0 for maxium & minium steering (+/-28 deg)
        if cmd_vel.angular.z > 1.0 : cmd_vel.angular.z = 1.0
        if cmd_vel.angular.z <-1.0 : cmd_vel.angular.z = -1.0
        cmd_steer = cmd_vel.angular.z * 2000
        
        # ---- NR steer  ------
        # As NR is is using negative "cmd_vel.linear.x" for gear and speed
        # the "break" will be controlled on cmd_vel.linear.z
        # if cmd_vel.linear.x = 0 and cmd_vel.linear.z = 0 the NR will be in neutral and just roll to a stop.
        if cmd_vel.linear.z > 0.01 :
            if cmd_vel.linear.z > 1 : cmd_vel.linear.z = 1 		# cmd_vel.linear.z normilized to 1.0 - 0 for maxium & minium (100 = full break)
            cmd_brake = cmd_vel.linear.z * 100
        else :
            cmd_brake = 0
        
        # --- Tidy & check data ---
        if cmd_speed > 1000 : cmd_speed = 1000
        if cmd_speed < 0 : cmd_speed = 0
        if cmd_steer > 2000 : cmd_steer = 2000
        if cmd_steer < -2000: cmd_steer = -2000
        if cmd_brake > 100 : cmd_brake = 100
        if cmd_brake < 0: cmd_brake = 0

        packet = [cmd_mode, cmd_e_stop, cmd_gear, cmd_speed, cmd_steer, cmd_brake]
#-----------------------------------------------------------------------

#=======================================================================
    def pub_cmd(self, test_case_number, serial_msg_hz, mode, e_stop, gear, speed, steer, brake) :
        ctrl_msg = CtrlCmd()
        ctrl_msg.test_case = test_case_number
        ctrl_msg.serial_msg_hz = serial_msg_hz
        ctrl_msg.mode = mode
        ctrl_msg.e_stop = e_stop
        ctrl_msg.gear = gear
        ctrl_msg.speed = speed
        ctrl_msg.steer = steer
        ctrl_msg.brake = brake
        ctrl_msg.alive = self.alive
        
        self.nr_cmd_pub.publish(ctrl_msg)
#-----------------------------------------------------------------------      
  
#=======================================================================
    def conversion(self, mode, e_stop, gear) :
        if mode == 0: 
            mode = "Manual"
        else:
            mode = "Auto"

        if e_stop == 0:
            e_stop = "OFF"
        else:
            e_stop = "ON"

        if gear == 0:
            gear = "Drive"
        elif gear == 2:
            gear = "Rear"
        else:
            gear = "Neutral"

        return mode, e_stop, gear
#-----------------------------------------------------------------------
        
#=======================================================================
    def sub_recv_status(self,RecvStatus) :
        #print("--------------------status(sub)--------------------")
        #print("recv_status(mode) : {}".format(RecvStatus.mode))
        #print("recv_status(e_stop) : {}".format(RecvStatus.e_stop))
        #print("recv_status(gear) : {}".format(RecvStatus.gear))
        #print("recv_status(speed) : {}".format(RecvStatus.speed))
        #print("recv_status(steer) : {}".format(RecvStatus.steer))
        #print("recv_status(brake) : {}".format(RecvStatus.brake))
        #print("recv_status(enc) : {}".format(RecvStatus.enc))
        #print("recv_status(batt) : {}".format(RecvStatus.batt))
        #print("recv_status(alive) : {}".format(RecvStatus.alive))
        #print("---------------------------------------------------\n")
        return
#-----------------------------------------------------------------------

#=======================================================================
    def main_loop(self, test_case_number, serial_msg_hz) :
        
        if debug: print(serial_msg_hz)
        
        rate = rospy.Rate(100)
        t_offset = time.time()
        t_prev = 0.0
        t = 0.0
        self.alive = 0
        while not rospy.is_shutdown(): 
            t = time.time() - t_offset
            t_elapsed = t - t_prev
 
            if t_elapsed > serial_msg_hz:               
                t_prev = t
                
 #               cmd_packet = self.test_case.ctrl_cmd_test_case(test_case_number, t, serial_msg_hz)
                cmd_packet = packet
                
                self.pub_cmd(
                    test_case_number,
                    serial_msg_hz,
                    cmd_packet[0],
                    cmd_packet[1],
                    cmd_packet[2],
                    cmd_packet[3],
                    cmd_packet[4],
                    cmd_packet[5],
                )
               
                self.alive = self.alive + 1
                if self.alive == 256:
                    self.alive = 0
            rate.sleep()
#----------------------------------------------------------------------- 

#=======================================================================
    def __init__(self):
        #self.test_case = test_case()
        rospy.init_node("nr_cmd", anonymous=True)
        self.serial_msg_hz = rospy.get_param('~serial_msg_hz', 1/20)
        self.scaler_nr_speed = rospy.get_param('~speed_scaler', 125)  	# 1000 / Max speed(8.2m/s) 
        self.baud = rospy.get_param('~baud', 115200)
        
        
        self.nr_cmd_pub = rospy.Publisher("/night_rider/ctrl_cmd", CtrlCmd, queue_size=1)
        self.nr_cmd_vel_sub = rospy.Subscriber("/night_rider/cmd_vel", Twist, self.cmd_vel_callback)
        self.nr_status_sub = rospy.Subscriber("/night_rider/recv_status", RecvStatus, self.sub_recv_status)
        self.main_loop(test_case_number, self.serial_msg_hz)
#-----------------------------------------------------------------------    

if __name__ == "__main__":
    try:
        test_case_number = int(input("Change : Mode(1), E-STOP(2), Gear(3), Speed(4), Steer(5), Speed and Steer(6) : "))
        if test_case_number >= 0 and test_case_number <= 6 :
            tester = nr_serial_test()
    except rospy.ROSInterruptException:
        pass
