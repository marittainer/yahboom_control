from Arm_Lib_New import Arm_Device1
import time
Arm = Arm_Device1()
time.sleep(.1)


## BOW
'''
Arm.Arm_serial_servo_write(2, 30,1500)
time.sleep(.1)
Arm.Arm_serial_servo_write(3, 50,1500)
time.sleep(.1)
Arm.Arm_serial_servo_write(4, 70,1500)
time.sleep(1)
Arm.Arm_serial_servo_write(6, 90, 500)
time.sleep(.75)
Arm.Arm_serial_servo_write(6,180,500)
time.sleep(.75)
Arm.Arm_serial_servo_write(5, 180,500)
time.sleep(.55)

# GO HOME
Arm.Arm_serial_servo_write(5,90,500)
time.sleep(1)
Arm.Arm_serial_servo_write(2, 90,1500)
time.sleep(.1)
Arm.Arm_serial_servo_write(3, 90,1500)
time.sleep(.1)
Arm.Arm_serial_servo_write(4, 90,1500)
time.sleep(3)
'''
#Go HOME
#Arm.Arm_serial_servo_write6_array([90, 70, 0, 0, 90, 180], 4000)

#Arm.Arm_serial_servo_write(3, 10, 500)
#Arm.Arm_serial_servo_write(1, 70, 500)
#time.sleep(2)
#Arm.Arm_serial_servo_write(3,75 ,500)
#time.sleep(3)
#[x1, y1, z1, ext] = Arm.get_pos_xyz_proj([90, 90, 90, 90, 90, 180])
#time.sleep(1)


#Arm.bus.write_i2c_block_data(Arm.addr, 0x10 + 2, [7, 132, 0, 10])
#Arm.Arm_serial_servo_write(4, 0, 500)
#print(Arm.Arm_serial_servo_read(1))

[theta1, theta2, theta3, theta4] =Arm.calc_theor_angles(0,0,48)
print(theta1, theta2, theta3, theta4)
time.sleep(1)
Arm.Arm_serial_servo_write6_array([theta1, theta2, theta3, theta4, 90,180], 4000)
time.sleep(5)
[x, y, z, ext] = Arm.get_pos_xyz()
print(x, ", ", y, ",", z)







