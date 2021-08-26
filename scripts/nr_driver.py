#!/usr/bin/env python
# -*- coding: utf-8 -*-

from std_msgs.msg import String, UInt8, Int16
from night_rider.msg import CtrlCmd, RecvStatus
import rospy
import rospkg
import message_filters
import numpy as np
import os
import serial
import struct
import time

debug = 0


class nr_driver:    
#=======================================================================
    def sub_ctrl_cmd(self, CtrlCmd):
        if not self.init_success:
            self.nr_serial(self.port, self.baud)

        self.send_ctrl_cmd(
            CtrlCmd.mode,
            CtrlCmd.e_stop,
            CtrlCmd.gear,
            CtrlCmd.speed,
            CtrlCmd.steer,
            CtrlCmd.brake,
            CtrlCmd.alive,
        )

        recv_packet = self.recv_serial_data()
        
        if debug :
            self.print_status( recv_packet[0], recv_packet[1], recv_packet[2], recv_packet[3], recv_packet[4], recv_packet[5], recv_packet[6], recv_packet[7], recv_packet[8] )
#-----------------------------------------------------------------------

#=======================================================================
    def nr_serial(self, serial_port, baudrate):
        self.ser = serial.Serial(serial_port, baudrate)
        self.ser.flushInput()
        self.ser.flushOutput()
        self.alive = 0
        self.init_success = True
#-----------------------------------------------------------------------
#=======================================================================
    def send_ctrl_cmd(self, cmd_mode, cmd_e_stop, cmd_gear, cmd_speed, cmd_steer, cmd_brake, cmd_alive):
        header = "STX".encode()
        tail = "\r\n".encode()
        cmd_speed = cmd_speed
        cmd_brake = cmd_brake
        data = struct.pack(
            ">BBBHhBB",
            cmd_mode,
            cmd_e_stop,
            cmd_gear,
            cmd_speed,
            cmd_steer,
            cmd_brake,
            cmd_alive,
        )
        packet = header + data + tail
        print(packet)
        print("---sent to NR---- mode: ", cmd_mode, "  stop: ", cmd_e_stop, "  gear: ", cmd_gear, "  speed: ", cmd_speed, "  steer ", cmd_steer, "  brake: ", cmd_brake)
        self.ser.write(packet)
        self.ser.flushOutput()
        
        return
        
#-----------------------------------------------------------------------
#=======================================================================
    def recv_serial_data(self):
        while True:
            packet = self.ser.readline()
            if not len(packet) == 20:
                packet_delay = self.ser.readline()
                packet = packet + packet_delay
            if len(packet) == 18 or len(packet) == 20:
                if len(packet) == 18:
                    fmt = "<BBBBBBhhBfHB"  # nr platform
                    # fmt = ">BBBBBBhhBfHB"  # serial test
                elif len(packet) == 20:
                    fmt = "<BBBBBBhhBfHBBB"  # nr platform
                    # fmt = ">BBBBBBhhBfHBBB"  # serial test
                data = struct.unpack(fmt, packet)
                header = packet[0:3].decode()
                if header == "STX":
                    status_mode = data[3]
                    status_e_stop = data[4]
                    status_gear = data[5]
                    status_speed = data[6]
                    status_steer = data[7]
                    status_brake = data[8]
                    status_enc = data[9]
                    status_batt = data[10]
                    status_alive = data[11]
                    recv_packet = [
                        status_mode,
                        status_e_stop,
                        status_gear,
                        status_speed,
                        status_steer,
                        status_brake,
                        status_enc,
                        status_batt,
                        status_alive,
                    ]
                    self.pub_status(
                        status_mode,
                        status_e_stop,
                        status_gear,
                        status_speed,
                        status_steer,
                        status_brake,
                        status_enc,
                        status_batt,
                        status_alive,
                    )
                    return recv_packet
#-----------------------------------------------------------------------
#=======================================================================
    def pub_status(self, mode, e_stop, gear, speed, steer, brake, enc, batt, alive):
        recv_status = RecvStatus()
        recv_status.mode = mode
        recv_status.e_stop = e_stop
        recv_status.gear = gear
        recv_status.speed = speed
        recv_status.steer = steer
        recv_status.brake = brake
        recv_status.enc = enc
        recv_status.batt = batt
        recv_status.alive = alive
        print(mode,e_stop,gear,speed,steer,brake,enc,batt,alive)
        self.nr_status_pub.publish(recv_status)
#-----------------------------------------------------------------------
#=======================================================================
    def print_status(self, mode, e_stop, gear, speed, steer, brake, enc, batt, alive):
        # os.system('clear')
        mode, e_stop, gear = self.conversion(mode, e_stop, gear)
        print("--------------------status(pub)--------------------")
        print("mode : {}".format(mode))
        print("e_stop : {}".format(e_stop))
        print("gear : {}".format(gear))
        nr_RPM = speed
        speed_value = speed / 200 * 1000
        print("speed_value(nr_RPM) : {}({})".format(speed_value, nr_RPM))
        nr_steer = steer / 71
        steer_value = steer
        print("steer_value(nr_STEER) : {}({})".format(steer_value, nr_steer))
        print("brake : {}".format(brake))
        print("enc : {}".format(enc))
        print("batt : {} ".format(batt))
        print("alive : {}".format(alive))
        print("---------------------------------------------------\n")
#-----------------------------------------------------------------------
#=======================================================================
    def conversion(self, mode, e_stop, gear):
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
    def main_loop(self):
        self.nr_cmd_sub = rospy.Subscriber("/night_rider/ctrl_cmd", CtrlCmd, self.sub_ctrl_cmd)
        self.nr_status_pub = rospy.Publisher("/night_rider/status", RecvStatus, queue_size=1)
        rospy.spin()
#-----------------------------------------------------------------------

#=======================================================================
    def __init__(self):
        rospy.init_node("nr_driver", anonymous=True)
        self.port = rospy.get_param('~port', '/dev/ttyS0')
        self.baud = rospy.get_param('~baud', 115200)
        self.init_success = False
        self.nr_serial(self.port, self.baud)
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.main_loop()
        self.ser.close()
        rate.sleep()
#-----------------------------------------------------------------------
if __name__ == "__main__":
    try:
        driver = nr_driver()
    except rospy.ROSInterruptException:
        pass
