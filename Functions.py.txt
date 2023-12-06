from hub import light_matrix
from hub import port
import runloop
import motor_pair
from hub import motion_sensor
import time
import motor, math

#מייצר זוג מנועים
motor_pair.pair(motor_pair.PAIR_1, port.C, port.D)

#משתנים
p_error = 0
error_sum = 0
kp = 1.5
kd = 0
ki = 0

#PID
def PID():
    global error_sum
    error = motion_sensor.tilt_angles()[0]
    P = error * kp
    error_sum += error
    I = error_sum * ki
    ad = error - p_error
    D = ad * kd
    PID = P + I + D
    return PID


#משהו שמסיע את הרובוט
def drive(cm:int, speed:int, angle:int):
    if cm == 0:
        return# Prevent division by zero error
    while (motor.relative_position(port.D) + (motor.relative_position(port.C)*-1)) / 360 * 2.7 * math.pi < cm:
        motor_pair.move_tank(motor_pair.PAIR_1, int(speed + PID()), int((speed - PID())))
    motor_pair.stop(motor_pair.PAIR_1)

#מסובב את הרובוט
def turn(angle):
    yaw = motion_sensor.tilt_angles()[0]
    while yaw - angle > 1:
        motor_pair.move_tank(motor_pair.PAIR_1, (angle - yaw) / 3 + 60, ((angle - yaw) / 3 + 60) * -1)
        yaw = motion_sensor.tilt_angles()[0]
    motor_pair.stop(motor_pair.PAIR_1)
motion_sensor.reset_yaw(0)



drive(5000, 500, 0)