#!/usr/bin/env python3
#coding: utf-8
import math
import smbus
import time
import numpy as np
from scipy.optimize import minimize
from scipy.optimize import fsolve
# V0.0.5

class Arm_Device1(object):

    def __init__(self):
        self.addr = 0x15
        self.bus = smbus.SMBus(1)
        self.l2 = 8.5
        self.l3 = 8.5
        self.l4 = 19.5
    #return array of all servo angles
    def pos_array(self):
        theta1 = self.Arm_serial_servo_read(1)
        theta2 = self.Arm_serial_servo_read(2)
        theta3 = self.Arm_serial_servo_read(3)
        theta4 = self.Arm_serial_servo_read(4)
        theta5 = self.Arm_serial_servo_read(5)
        theta6 = self.Arm_serial_servo_read(6)
        return [theta1, theta2, theta3, theta4, theta5, theta6]
    # ensure robot will not damage itself in executing angle
    def checklim(self, id, angle):
        joints = self.pos_array()
        joints[id-1] = angle
        [x, y, z, ext] = self.get_pos_xyz_proj(joints)
        # do not allow end effector to hit table
        if z < 0:
            return 0
        #do not allow arm to hit pi/board/other hardware
        if (joints[1] + joints[2]) > 270:
            return 0
        if x < 0 and (abs(y) < 9 and z < 11 and abs(ext) < 20):
            return 0
        return 1
        
    def checklim6(self, joints):
        [x, y, z, ext] = self.get_pos_xyz_proj(joints)
        # do not allow end effector to hit table
        if z < 0:
            return 0
        #do not allow arm to hit pi/board/other hardware
        if (joints[1] + joints[2]) > 270:
            return 0
        if x < 0 and (abs(y) < 9 and z < 11 and abs(ext) < 20):
            print(ext)
            return 0
        return 1



    # Set the bus servo angle interface: id: 1-6 (0 means sending 6 servos) angle: 0-180 Set the angle to which the servo will move.
    def Arm_serial_servo_write(self, id, angle, time):
        allGood = self.checklim(id, angle)
        angleOrig = angle
        if not allGood:
            print("position out of range")
            return
        if id == 2 or id == 3 or id == 4 or id == 5 or id == 6 or id ==1:
            if angle < 0 or angle > 180:
                print("angle outside of range (0-180 deg)")
                return 
        if id == 0:  # This is all servo controls
            self.Arm_serial_servo_write6(angle, angle, angle, angle, angle, angle, time)
        elif id == 2 or id == 3 or id == 4:  # opposite angle to reality
            angle = 180 - angle
            pos = int((3100 - 900) * (angle - 0) / (180 - 0) + 900)
            # pos = ((pos << 8) & 0xff00) | ((pos >> 8) & 0xff)
            value_H = (pos >> 8) & 0xFF
            value_L = pos & 0xFF
            time_H = (time >> 8) & 0xFF
            time_L = time & 0xFF
            #try
            self.bus.write_i2c_block_data(self.addr, 0x10 + id, [value_H, value_L, time_H, time_L])
            self.pos_feedback_adjust(id, angleOrig)
            #except:
            #    print('Arm_serial_servo_write I2C error')
        elif id == 5:
            pos = int((3700 - 380) * (angle - 0) / (270 - 0) + 380)
            # pos = ((pos << 8) & 0xff00) | ((pos >> 8) & 0xff)
            value_H = (pos >> 8) & 0xFF
            value_L = pos & 0xFF
            time_H = (time >> 8) & 0xFF
            time_L = time & 0xFF
            try:
                self.bus.write_i2c_block_data(self.addr, 0x10 + id, [value_H, value_L, time_H, time_L])
                self.pos_feedback_adjust(id, angleOrig)
            except:
                print('Arm_serial_servo_write I2C error')
        else:
            pos = int((3100 - 900) * (angle - 0) / (180 - 0) + 900)
            # pos = ((pos << 8) & 0xff00) | ((pos >> 8) & 0xff)
            value_H = (pos >> 8) & 0xFF
            value_L = pos & 0xFF
            time_H = (time >> 8) & 0xFF
            time_L = time & 0xFF
            try:
                self.bus.write_i2c_block_data(self.addr, 0x10 + id, [value_H, value_L, time_H, time_L])
                self.pos_feedback_adjust(id, angleOrig)
            except:
                print('Arm_serial_servo_write I2C error')

    # Set any bus servo angle interface: id: 1-250 (0 is group transmission) angle: 0-180 means 900 3100 0 - 180
    def Arm_serial_servo_write_any(self, id, angle, time):
        if id != 0:
            pos = int((3100 - 900) * (angle - 0) / (180 - 0) + 900)
            value_H = (pos >> 8) & 0xFF
            value_L = pos & 0xFF
            time_H = (time >> 8) & 0xFF
            time_L = time & 0xFF
            try:
                self.bus.write_i2c_block_data(self.addr, 0x19, [id & 0xff, value_H, value_L, time_H, time_L])
            except:
                print('Arm_serial_servo_write_any I2C error')
        elif id == 0:  #This is all servo controls 
            pos = int((3100 - 900) * (angle - 0) / (180 - 0) + 900)
            # pos = ((pos << 8) & 0xff00) | ((pos >> 8) & 0xff)
            value_H = (pos >> 8) & 0xFF
            value_L = pos & 0xFF
            time_H = (time >> 8) & 0xFF
            time_L = time & 0xFF
            try:
                self.bus.write_i2c_block_data(self.addr, 0x17, [value_H, value_L, time_H, time_L])
            except:
                print('Arm_serial_servo_write_any I2C error')

    # Set the bus servo neutral offset with one click, power on and move to the neutral position, and then send the following function, id: 1-6 (setting), 0 (restore to initial)
    def Arm_serial_servo_write_offset_switch(self, id):
        try:
            if id > 0 and id < 7:
                self.bus.write_byte_data(self.addr, 0x1c, id)
            elif id == 0:
                self.bus.write_byte_data(self.addr, 0x1c, 0x00)
                time.sleep(.5)
        except:
            print('Arm_serial_servo_write_offset_switch I2C error')

    #Set the status of the bus servo mid-bit offset. 0 means that the corresponding servo ID cannot be found, 1 means success, and 2 means failure is out of range.
    def Arm_serial_servo_write_offset_state(self):
        try:
            self.bus.write_byte_data(self.addr, 0x1b, 0x01)
            time.sleep(.001)
            state = self.bus.read_byte_data(self.addr, 0x1b)
            return state
        except:
            print('Arm_serial_servo_write_offset_state I2C error')
        return None

    #Set the bus servo angle interface: array
    #write seperate angles to all servos simultaneously
    def Arm_serial_servo_write6_array(self, joints, time):
        allGood = self.checklim6(joints)
        if not allGood:
            print("position out of range")
            return 
        s1, s2, s3, s4, s5, s6 = joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]
        if s1 > 180 or s2 > 180 or s3 > 180 or s4 > 180 or s5 > 270 or s6 > 180:
            print("The parameter input range is not within range 0-180！")
            return
        #try:
        pos = int((3100 - 900) * (s1 - 0) / (180 - 0) + 900)
        value1_H = (pos >> 8) & 0xFF
        value1_L = pos & 0xFF

        s2 = 180 - s2
        pos = int((3100 - 900) * (s2 - 0) / (180 - 0) + 900)
        value2_H = (pos >> 8) & 0xFF
        value2_L = pos & 0xFF

        s3 = 180 - s3
        pos = int((3100 - 900) * (s3 - 0) / (180 - 0) + 900)
        value3_H = (pos >> 8) & 0xFF
        value3_L = pos & 0xFF

        s4 = 180 - s4
        pos = int((3100 - 900) * (s4 - 0) / (180 - 0) + 900)
        value4_H = (pos >> 8) & 0xFF
        value4_L = pos & 0xFF

        pos = int((3700 - 380) * (s5 - 0) / (270 - 0) + 380)
        value5_H = (pos >> 8) & 0xFF
        value5_L = pos & 0xFF

        pos = int((3100 - 900) * (s6 - 0) / (180 - 0) + 900)
        value6_H = (pos >> 8) & 0xFF
        value6_L = pos & 0xFF
        time_H = (time >> 8) & 0xFF
        time_L = time & 0xFF

        data = [value1_H, value1_L, value2_H, value2_L, value3_H, value3_L,
                value4_H, value4_L, value5_H, value5_L, value6_H, value6_L]
        timeArr = [time_H, time_L]
        s_id = 0x1d
        self.bus.write_i2c_block_data(self.addr, 0x1e, timeArr)
        self.bus.write_i2c_block_data(self.addr, s_id, data)
        range = [0,1,2,3,4]
        #for x in range:
            #self.pos_feedback_adjust(x+1, joints[x])

       # except:
        #    print('Arm_serial_servo_write6 I2C error')

    # Set the bus servo angle interface: s1~S4 and s6: 0-180, S5: 0~270, time is the running time
    def Arm_serial_servo_write6(self, s1, s2, s3, s4, s5, s6, time):
        if s1 > 180 or s2 > 180 or s3 > 180 or s4 > 180 or s5 > 270 or s6 > 180:
            print("The parameter input range is not within 0-180!")
            return
        try:
            pos = int((3100 - 900) * (s1 - 0) / (180 - 0) + 900)
            value1_H = (pos >> 8) & 0xFF
            value1_L = pos & 0xFF

            s2 = 180 - s2
            pos = int((3100 - 900) * (s2 - 0) / (180 - 0) + 900)
            value2_H = (pos >> 8) & 0xFF
            value2_L = pos & 0xFF

            s3 = 180 - s3
            pos = int((3100 - 900) * (s3 - 0) / (180 - 0) + 900)
            value3_H = (pos >> 8) & 0xFF
            value3_L = pos & 0xFF

            s4 = 180 - s4
            pos = int((3100 - 900) * (s4 - 0) / (180 - 0) + 900)
            value4_H = (pos >> 8) & 0xFF
            value4_L = pos & 0xFF

            pos = int((3700 - 380) * (s5 - 0) / (270 - 0) + 380)
            value5_H = (pos >> 8) & 0xFF
            value5_L = pos & 0xFF

            pos = int((3100 - 900) * (s6 - 0) / (180 - 0) + 900)
            value6_H = (pos >> 8) & 0xFF
            value6_L = pos & 0xFF
            time_H = (time >> 8) & 0xFF
            time_L = time & 0xFF

            data = [value1_H, value1_L, value2_H, value2_L, value3_H, value3_L,
                    value4_H, value4_L, value5_H, value5_L, value6_H, value6_L]
            timeArr = [time_H, time_L]
            s_id = 0x1d
            self.bus.write_i2c_block_data(self.addr, 0x1e, timeArr)
            self.bus.write_i2c_block_data(self.addr, s_id, data)
        except:
            print('Arm_serial_servo_write6 I2C error')

    # Read the specified servo angle, id: 1-6, return 0-180, read error return None
    def Arm_serial_servo_read(self, id):
        if id < 1 or id > 6:
            print("id must be 1 - 6")
            return None
        try:
            self.bus.write_byte_data(self.addr, id + 0x30, 0x0)
            time.sleep(0.003)
            pos = round(self.bus.read_word_data(self.addr, id + 0x30),4)
            #print(pos)
        except:
            print('Arm_serial_servo_read I2C error')
            return None
        if pos == 0:
            return None
        pos = int(pos >> 8 & 0xff) | (pos << 8 & 0xff00)
        #print(pos)
        if id == 5:
            pos = round((270 - 0) * (pos - 380) / (3700 - 380) + 0, 2)
            if pos > 270 or pos < 0:
                return None
        else:
            pos = round((180 - 0) * (pos - 900) / (3100 - 900) + 0, 2)
 #           if pos > 181 or pos < -1:
 #               return None
        if id == 2 or id == 3 or id == 4:
            pos = round(180 - pos,2)
        # print(pos)
        return pos

    # Read the bus servo angle, id: 1-250, return 0-180
    def Arm_serial_servo_read_any(self, id):
        if id < 1 or id > 250:
            print("id must be 1 - 250")
            return None
        try:
            self.bus.write_byte_data(self.addr, 0x37, id)
            time.sleep(0.003)
            pos = self.bus.read_word_data(self.addr, 0x37)
        except:
            print('Arm_serial_servo_read_any I2C error')
            return None
        # print(pos)
        pos = (pos >> 8 & 0xff) | (pos << 8 & 0xff00)
        # print(pos)
        pos = int((180 - 0) * (pos - 900) / (3100 - 900) + 0)
        # print(pos)
        return pos

    # Read the servo status, return 0xda normally, return 0x00 if no data can be read, other values ​​​​are servo errors.
    def Arm_ping_servo(self, id):
        data = int(id)
        if data > 0 and data <= 250:
            reg = 0x38
            self.bus.write_byte_data(self.addr, reg, data)
            time.sleep(.003)
            value = self.bus.read_byte_data(self.addr, reg)
            times = 0
            while value == 0 and times < 5:
                self.bus.write_byte_data(self.addr, reg, data)
                time.sleep(.003)
                value = self.bus.read_byte_data(self.addr, reg)
                times += 1
                if times >= 5:
                    return None
            return value
        else:
            return None

    # Read hardware version number
    def Arm_get_hardversion(self):
        try:
            self.bus.write_byte_data(self.addr, 0x01, 0x01)
            time.sleep(.001)
            value = self.bus.read_byte_data(self.addr, 0x01)
        except:
            print('Arm_get_hardversion I2C error')
            return None
        version = str(0) + '.' + str(value)
        # print(version)
        return version

    #Torque switch 1: Open torque 0: Close torque (can be turned)
    def Arm_serial_set_torque(self, onoff):
        try:
            if onoff == 1:
                self.bus.write_byte_data(self.addr, 0x1A, 0x01)
            else:
                self.bus.write_byte_data(self.addr, 0x1A, 0x00)
        except:
            print('Arm_serial_set_torque I2C error')

    # Set the bus servo number
    def Arm_serial_set_id(self, id):
        try:
            self.bus.write_byte_data(self.addr, 0x18, id & 0xff)
        except:
            print('Arm_serial_set_id I2C error')

    # Set the current product color to 1~6, and the RGB light will turn on corresponding color.
    def Arm_Product_Select(self, index):
        try:
            self.bus.write_byte_data(self.addr, 0x04, index & 0xff)
        except:
            print('Arm_Product_Select I2C error')

    # Set RGB lights to specify colors
    def Arm_RGB_set(self, red, green, blue):
        try:
            self.bus.write_i2c_block_data(self.addr, 0x02, [red & 0xff, green & 0xff, blue & 0xff])
        except:
            print('Arm_RGB_set I2C error')

    #Set K1 button mode, 0: default mode 1: learning mode
    def Arm_Button_Mode(self, mode):
        try:
            self.bus.write_byte_data(self.addr, 0x03, mode & 0xff)
        except:
            print('Arm_Button_Mode I2C error')

    # Restart the driver board
    def Arm_reset(self):
        try:
            self.bus.write_byte_data(self.addr, 0x05, 0x01)
        except:
            print('Arm_reset I2C error')

    #PWD servo control id:1-6 (0 controls all servos) angle: 0-180
    def Arm_PWM_servo_write(self, id, angle):
        try:
            if id == 0:
                self.bus.write_byte_data(self.addr, 0x57, angle & 0xff)
            else:
                self.bus.write_byte_data(self.addr, 0x50 + id, angle & 0xff)
        except:
            print('Arm_PWM_servo_write I2C error')

    # Clear action group
    def Arm_Clear_Action(self):
        try:
            self.bus.write_byte_data(self.addr, 0x23, 0x01)
            time.sleep(.5)
        except:
            print('Arm_Clear_Action I2C error')

    #In learning mode, record the current action once
    def Arm_Action_Study(self):
        try:
            self.bus.write_byte_data(self.addr, 0x24, 0x01)
        except:
            print('Arm_Action_Study I2C error')

    # Action group operation mode 0: Stop operation 1: Single operation 2: Cycle operation
    def Arm_Action_Mode(self, mode):
        try:
            self.bus.write_byte_data(self.addr, 0x20, mode & 0xff)
        except:
            print('Arm_Clear_Action I2C error')

    # Read the number of saved action groups
    def Arm_Read_Action_Num(self):
        try:
            self.bus.write_byte_data(self.addr, 0x22, 0x01)
            time.sleep(.001)
            num = self.bus.read_byte_data(self.addr, 0x22)
            return num
        except:
            print('Arm_Read_Action_Num I2C error')

    # Turn on the buzzer, delay defaults to 0xff, and the buzzer keeps sounding.
    # delay=1~50, after turning on the buzzer, it will automatically turn off after delay*100 milliseconds. The maximum delay time is 5 seconds.
    def Arm_Buzzer_On(self, delay=0xff):
        try:
            if delay != 0:
                self.bus.write_byte_data(self.addr, 0x06, delay&0xff)
        except:
            print('Arm_Buzzer_On I2C error')

    # Turn off the buzzer
    def Arm_Buzzer_Off(self):
        try:
            self.bus.write_byte_data(self.addr, 0x06, 0x00)
        except:
            print('Arm_Buzzer_Off I2C error')

    def bus_servo_control(self, id, num, time=1000):
        try:
            # if num > 4000 or num < 96:
            #     print("bus_servo_control error, num must be [96, 4000]")
            #     return

            time_H = (time >> 8) & 0xFF
            time_L = time & 0xFF
        
            if id == 1 or id == 6:
                if num > 3100 or num < 900:
                    print("bus_servo_control error, num must be [900, 3100]")
                    return
                pos = int(num)
                value_H = (pos >> 8) & 0xFF
                value_L = pos & 0xFF
            elif id == 2 or id == 3 or id == 4:  # 与实际相反角度
                if num > 3100 or num < 900:
                    print("bus_servo_control error, num must be [900, 3100]")
                    return
                pos = int(3100 - num + 900)
                value_H = (pos >> 8) & 0xFF
                value_L = pos & 0xFF
            elif id == 5:
                if num > 4200 or num < 900:
                    print("bus_servo_control error, num must be [900, 4200]")
                    return
                pos = int(num - 514)
                value_H = (pos >> 8) & 0xFF
                value_L = pos & 0xFF
            else:
                print("bus_servo_control error, id must be [1, 6]")
                return
            self.bus.write_i2c_block_data(self.addr, 0x10 + id, [value_H, value_L, time_H, time_L])
        except:
            print('bus_servo_control error')

    def __change_value(self, value):
        try:
            val = 3100 - int(value) + 900
            return int(val)
        except:
            return None

    def bus_servo_control_array6(self, array, time=1000):
        try:
            if len(array) != 6:
                print("bus_servo_control_array6 input error")
                return

            s1, s2, s3, s4, s5, s6 = array[0], array[1], array[2], array[3], array[4], array[5]
            if s1 > 3100 or s2 > 3100 or s3 > 3100 or s4 > 3100 or s5 > 4200 or s6 > 3100:
                print("bus_servo_control_array6 input error")
                return
            elif s1 < 900 or s2 < 900 or s3 < 900 or s4 < 900 or s5 < 900 or s6 < 900:
                print("bus_servo_control_array6 input error")
                return

            pos = int(s1)
            value1_H = (pos >> 8) & 0xFF
            value1_L = pos & 0xFF

            s2 = self.__change_value(s2)
            pos = int(s2)
            value2_H = (pos >> 8) & 0xFF
            value2_L = pos & 0xFF

            s3 = self.__change_value(s3)
            pos = int(s3)
            value3_H = (pos >> 8) & 0xFF
            value3_L = pos & 0xFF

            s4 = self.__change_value(s4)
            pos = int(s4)
            value4_H = (pos >> 8) & 0xFF
            value4_L = pos & 0xFF

            s5 = s5 - 514
            pos = int(s5)
            value5_H = (pos >> 8) & 0xFF
            value5_L = pos & 0xFF

            pos = int(s6)
            value6_H = (pos >> 8) & 0xFF
            value6_L = pos & 0xFF

            time_H = (time >> 8) & 0xFF
            time_L = time & 0xFF

            data = [value1_H, value1_L, value2_H, value2_L, value3_H, value3_L,
                    value4_H, value4_L, value5_H, value5_L, value6_H, value6_L]
            timeArr = [time_H, time_L]
            s_id = 0x1d
            self.bus.write_i2c_block_data(self.addr, 0x1e, timeArr)
            self.bus.write_i2c_block_data(self.addr, s_id, data)
        except:
            print('bus_servo_control_array6 I2C error')
    def cos_deg(self,angle):
        angle = math.radians(angle)
        return math.cos(angle)
    def sin_deg(self,angle):
        angle = math.radians(angle)
        return math.sin(angle)

    #calculate present position in space (cm)
    def get_pos_xyz(self):
        l2 = self.l2
        l3 = self.l3
        l4 = self.l4

        theta1 = self.Arm_serial_servo_read(1)
        theta2 = self.Arm_serial_servo_read(2)
        theta3 = self.Arm_serial_servo_read(3)
        theta4 = self.Arm_serial_servo_read(4)
        #print(theta1, " ", theta2, " ", theta3, " ", theta4, " ")

        theta3 = theta3 - 90
        theta4 = theta4 - 90

        ext = (l2*self.cos_deg(theta2) + l3*self.cos_deg(theta2 + theta3) + l4*self.cos_deg(theta2 + theta3 +  theta4))
        z = round(l2*self.sin_deg(theta2) + l3*self.sin_deg(theta2 + theta3) + l4*self.sin_deg(theta2 + theta3 + theta4) + 12.9, 2)
        x = round(ext*self.sin_deg(theta1),2)
        y = round(ext*self.cos_deg(theta1),2)

        return [x, y, z, ext]

    #calculate hypothetical position in space (cm)
    
    def get_pos_xyz_proj(self, joints):
        
        l2 = self.l2
        l3 = self.l3
        l4 = self.l4
        theta1 = joints[0]
        theta2 = joints[1]
        theta3 = joints[2]
        theta4 = joints[3]
        #print(theta1, theta2, theta3, theta4)
        theta3 = theta3 - 90
        theta4 = theta4 - 90
        ext = (l2*self.cos_deg(theta2) + l3*self.cos_deg(theta2 + theta3) + l4*self.cos_deg(theta2 + theta3 +  theta4))
        z = round(l2*self.sin_deg(theta2) + l3*self.sin_deg(theta2 + theta3) + l4*self.sin_deg(theta2 + theta3 + theta4) + 12.9, 2)
        x = round(ext*self.sin_deg(theta1),2)
        y = round(ext*self.cos_deg(theta1),2)
        #print("proj x", x, "proj y", y,"proj z", z)
        return [x, y, z, ext]
    #adjust servo position with potentiometer feedback
    
    def pos_feedback_adjust(self, id, angle):
        #print("adjusting")
        time1 = 1
        pos1 = self.Arm_serial_servo_read(id)
        time.sleep(.5)
        pos2 = self.Arm_serial_servo_read(id)
        time.sleep(.5)
        
        #wait until servo is done moving
        while pos1 != pos2:
            time.sleep(.5)
            pos1 = self.Arm_serial_servo_read(id)
            time.sleep(.5)
            pos2 = self.Arm_serial_servo_read(id)
            time.sleep(.5)
        
        
        if (pos2 - angle) > .55:
            print("overshoot")
            new_ang = (angle - (pos2-angle))
            new_ang = 180 - new_ang
            pos = int((3100 - 900) * (new_ang - 0) / (180 - 0) + 900)
            value_H = (pos >> 8) & 0xFF
            value_L = pos & 0xFF
            time_H = (time1 >> 8) & 0xFF
            time_L = time1 & 0xFF
            self.bus.write_i2c_block_data(self.addr, 0x10 + id, [value_H, value_L, time_H, time_L])
            self.pos_feedback_adjust(id, angle)

        elif (angle - pos2) > .55:
            print("undershoot")
            new_ang = (angle + (angle - pos2))
            print(new_ang)
            new_ang = 180 - new_ang
            pos= int((3100 - 900) * (new_ang - 0) / (180 - 0) + 900)
            value_H = (pos >> 8) & 0xFF
            value_L = pos & 0xFF
            time_H = (time1 >> 8) & 0xFF
            time_L = time1 & 0xFF
            self.bus.write_i2c_block_data(self.addr, 0x10 + id, [value_H, value_L, time_H, time_L])
            self.pos_feedback_adjust(id, angle)
        else:
            #print("all good") 
            return
            #inverse kinematics
        
    def calc_theor_angles(self, x, y, z):
        #[theta1, theta2, theta3, theta4, theta5, theta6] = self.pos_array()
        # Define the objective function to minimize deviation from 90 degrees

        def objective(vars):
            theta2, theta3, theta4 = vars
            # Minimize the sum of squared deviations from 90 degrees (in radians)
            return (theta2 - np.pi/2)**2 + (theta3)**2 + (theta4)**2
        
        # Define the system of equations as  
        def eq1(vars):
            theta2, theta3, theta4 = vars
            return l2 * np.cos(theta2) + l3 * np.cos(theta2 + theta3) + l4 * np.cos(theta2 + theta3 + theta4) - ext

        def eq2(vars):
            theta2, theta3, theta4 = vars
            return l2 * np.sin(theta2) + l3 * np.sin(theta2 + theta3) + l4 * np.sin(theta2 + theta3 + theta4) + 11.5 - z
        
        # Define the constraint dictionary for minimize
        cons = ({'type': 'eq', 'fun': eq1},{'type': 'eq', 'fun': eq2})

        # Define the bounds for theta2, theta3, and theta4 (0 to 180 degrees, converted to radians)
        bounds = [(-np.pi/2 - .5, np.pi/2 + .5), (-np.pi -.5, .5), (-np.pi -.5, .5)]

        #define link lengths
        l2 = self.l2
        l3 = self.l3
        l4 = self.l4

        #calc forward arm extension
        ext = math.sqrt(math.pow(x,2)+math.pow(y,2))

        #calculate theta1
        if x>0:
            theta1 = math.degrees(math.atan((y/x)))
        else:
            theta1 = 90

        #solve for theta2 - theta4
        initial_guess = [np.pi/4, np.pi/4, np.pi/4]
        result = minimize(objective, initial_guess, constraints=cons, bounds = bounds, method='SLSQP')

        if result.success:
            theta2, theta3, theta4 = np.degrees(result.x)
            theta3 = theta3 + 90
            theta4 = theta4 + 90
            return [theta1, theta2, theta3, theta4]
        else:
            print("Position cannot be reached", result.message)
            return
    
            
        
        
        

    
        
        






