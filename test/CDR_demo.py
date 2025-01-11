from Arm_Lib_New import Arm_Device1
import time
Arm = Arm_Device1()
time.sleep(.1)

#go home
Arm.Arm_serial_servo_write6(90,90,90,90,90,180, 4000)
time.sleep(4.5)

#send to block position
Arm.Arm_serial_servo_write(6,90,1000)
time.sleep(1.5)
Arm.Arm_serial_servo_write6(90,27,54,55,90,90,4000)
time.sleep(4.5)

[x, y, z, ext] = Arm.get_pos_xyz()
positions = Arm.pos_array()

print("position in cm.")
print("x:", x, "y:", y, "z:",z)

print("theta1: ",positions[0],"theta2:",positions[1],"theta3:",positions[2],"theta4:", positions[3], "theta5:",positions[4],"theta6:",positions[5])

#pick up block
time.sleep(1.5)
Arm.Arm_serial_servo_write6(90,27,54,55,90,137, 1000)
time.sleep(1.5)
Arm.Arm_serial_servo_write6(90,90,90,90,90,137,4000)
time.sleep(4.5)

[x, y, z, ext] = Arm.get_pos_xyz()
positions = Arm.pos_array()
print(" ")
print("position in cm.")
print("x:", x, "y:", y, "z:",z)
print("theta1: ",positions[0],"theta2:",positions[1],"theta3:",positions[2],"theta4:", positions[3], "theta5:",positions[4],"theta6:",positions[5])

Arm.Arm_serial_servo_write6(90,27,54,55,90,137, 4000)
time.sleep(4.5)
Arm.Arm_serial_servo_write6(90, 27, 54, 55, 90,90,1000)
time.sleep(1.5)
Arm.Arm_serial_servo_write6(90,90,90,90,90,180,4000)