#!/usr/bin/env python
# -*- coding: utf-8 -*-

#-------------- erp42_indro_test_driver.py ------------------------------------------------
# Arthor: Arron Griffiths | Company: Indro Robotics | Location: Ottawa, ON, Canada
# Contact: agriffiths@indrorobotics.com  
# Info: This is a simple python ros node that write serial commamds to the the wego erp4
#---------------------------------------------------------------------------------------------------

import rospy
import serial
import sensor_msgs
import std_msgs
import struct
from std_srvs.srv import SetBool, SetBoolResponse
import time


class erp42_indro_test_driver:
    def __init__(self):
#-------------- Serial setup --------------------------------------------------
        self.port = rospy.get_param('~port', '/dev/ttyS0')
        self.baud = rospy.get_param('~baud', 115200)
        self.serial_port = serial.Serial(self.port, self.baud) #timeout=1, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)
        self.serial_port.flushInput()
        self.serial_port.flushOutput()
        self.alive = 0
        self.init_success = True

#-------------- function deleceration -----------------------------------------
        rospy.Service('erp42_test_service', SetBool, self.erp42_test)
#-------------- ROS Spin loop -------------------------------------------------
        while not rospy.is_shutdown():
            rospy.spin()

        self.serial_port.close()
        
    def send_ctrl_cmd(self, cmd_mode, cmd_e_stop, cmd_gear, cmd_speed, cmd_steer, cmd_brake, cmd_alive):    
        header = "STX".encode()
        tail = "\r\n".encode()
        cmd_speed = cmd_speed
        data = struct.pack(
            ">BBBHhBB",
            cmd_mode,   # 0 = Auto/robot, 1 = Manual/controller
            cmd_e_stop, # 0 = Stop enabled, 1 = Stop Off (2~255 also make the eStop enabled)
            cmd_gear,   # 0 = Forward drive, 1 = Neural drive, 2 = Backwards (3~225 also make the platform go into Neutral)
            cmd_speed,  # 0~1000 speed (50 = 5km, 100 = 10km, ?? )
            cmd_steer,  # -2000 = left 28 deg, 0 = 0 deg turn, 2000 = right 28 deg 
            cmd_brake,  # 0 = No brake, 100 = full brake (10% error)
            cmd_alive,  # Set to zero on each message set 
        )
        packet = header + data + tail
        print(packet)
        self.serial_port.write(packet)
        self.serial_port.flushOutput()
        return   
               
    def pack_ctrl_cmd(self, cmd_mode, cmd_e_stop, cmd_gear, cmd_speed, cmd_steer, cmd_brake, cmd_alive, num_secs):
                
        time_end = time.time() + num_secs    
        
        rate = rospy.Rate(1000)
        t_offset = time.time()
        t_prev = 0.0
        time_loop = 0.0
        loop_hertz = 1/20
        
        #----------- Check value is within range -------
        if cmd_mode < 0 or cmd_mode > 1 : cmd_mode = 1            # if ERROR force to Manual mode
        if cmd_e_stop < 0 or cmd_e_stop > 255 : cmd_e_stop = 1    # if ERROR force to enable e stop
        if cmd_gear < 0 or cmd_gear > 2 : cmd_gear = 1            # if ERROR force to neutral mode
        if cmd_speed < 0 or cmd_speed > 1000 : cmd_speed =  0     # if ERROR force to stop
        if cmd_steer < -2000 or cmd_steer > 2000 : cmd_steer = 0  # if ERROR force steer wheel to 0 deg
        if cmd_brake < 0 or cmd_brake > 100 : cmd_brake = 100     # if ERROR force to stop
        if cmd_alive > 0 : alive = 0                              # Set alive to zero on each send               
        if num_secs > 30 : num_secs = 30                          # ensure serivce call is < 30 secs each time its called
        
        cmd_gear = cmd_gear * 10
        cmd_speed = cmd_speed * 71
        
        while not rospy.is_shutdown() and time.time() < time_end :
            time_loop = time.time() - t_offset
            t_elapsed = time_loop - t_prev
            if t_elapsed > loop_hertz:
                self.send_ctrl_cmd(cmd_mode, cmd_e_stop, cmd_gear, cmd_speed, cmd_steer, cmd_brake, cmd_alive)
                cmd_alive = cmd_alive + 1
                if cmd_alive == 256:
                    cmd_alive = 0
            
            rate.sleep()
        
        print("end of sending")        
        response = "Serial msg sent for " + str(num_secs) + " secs"
        return SetBoolResponse(True, response)
        
    def erp42_test(self, req):
        return self.pack_ctrl_cmd(0, 0, 1, 0, 1000, 50, 0, 10)  # <----------------------- Change numbers here to match the Protocol -------

if __name__ == "__main__":
    rospy.init_node('erp42_indro_test_driver')
    nri = erp42_indro_test_driver()
