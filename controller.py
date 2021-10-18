from steppyr import StepperController, DIRECTION_CW, DIRECTION_CCW
from steppyr.profiles.accel import AccelProfile
from steppyr.drivers.stepdir import StepDirDriver
from constant import *
import serial, time
import asyncio
from contextlib import suppress
import math
from tof10120 import read_filtered_distance
from avoidance import AvoidanceAction, tof10120_judgment
from smbus import SMBus
from enum import Enum
import RPi.GPIO as GPIO

avg_count = 0
avg_last = 0
avg_sum = 0
avg_cur = 0
last_angle = None
last_angle_fail = 0

stp_left = StepperController(
    StepDirDriver(6, 5),
    AccelProfile(),
)

stp_right = StepperController(
    StepDirDriver(24, 23),
    AccelProfile(),
)

stp_left.activate()
stp_right.activate()
stp_left.set_target_acceleration(ACCELER)
stp_left.set_target_speed(MAX_SPEED)
stp_right.set_target_acceleration(ACCELER)
stp_right.set_target_speed(MAX_SPEED)

## State Machine
class CarStatus(Enum):
    STANDBY = 0
    FOLLOWING = 1
    AVOID = 2
    SPINNING = 3

    def __str__(self):
        if self == CarStatus.STANDBY:
            return "StandBy"
        elif self == CarStatus.FOLLOWING:
            return "Following"
        elif self == CarStatus.AVOID:
            return "Avoid"
        elif self == CarStatus.SPINNING:
            return "Spinning"
        else:
            return "Unknown"

## Helper functions
def car_spin_around(angual, speed=None):
    if speed != None:
        stp_left.set_target_speed(speed)
        stp_right.set_target_speed(speed)
    step_to_spin = int((SPIN_AROUND_DIST * (angual * SPIN_BUFFER / 180) * PPR) / (WHEEL_DIAM * math.pi))
    stp_left.move(-step_to_spin)
    stp_right.move(step_to_spin)

########################

def ctrl_dir(ang):
    if ang <= -80 or ang >=80:
        speed_factor = TURNING_ANGLE_PN80
    elif ang <= -70 or ang >= 70:
        speed_factor = TURNING_ANGLE_PN70
    elif ang <= -60 or ang >= 60:
        speed_factor = TURNING_ANGLE_PN60
    elif ang <= -50 or ang >= 50:
        speed_factor = TURNING_ANGLE_PN50
    elif ang <= -40 or ang >= 40:
        speed_factor = TURNING_ANGLE_PN40
    elif ang <= -30 or ang >= 30:
        speed_factor = TURNING_ANGLE_PN30
    elif ang <= -20 or ang >= 20:
        speed_factor = TURNING_ANGLE_PN20
    elif ang <= -8 or ang >= 8:
        speed_factor = TURNING_ANGLE_PN8
    else:
        speed_factor = 1

    if ang > 0 : #人在左邊->左轉
        return {'left':speed_factor, 'right':1.0}
    else: #人在右邊->右轉
        return {'left':1.0, 'right':speed_factor}

def tof_avoid_control(action: AvoidanceAction):
    stp_left.stop()
    stp_right.stop()
    stp_left.set_current_steps(0)
    stp_right.set_current_steps(0)
    stp_left.set_target_acceleration(TOF_ACCELER)
    stp_right.set_target_acceleration(TOF_ACCELER)
    if action == AvoidanceAction.SPIN_LEFT or action == AvoidanceAction.SPIN_RIGHT: #向左自轉or向右自轉 避障
        if action == AvoidanceAction.SPIN_LEFT:
            car_spin_around(TOF_SPIN_ANG, TOF_SPEED)
        else:
            car_spin_around(-TOF_SPIN_ANG, TOF_SPEED)
    elif action == AvoidanceAction.BACK_LEFT or action == AvoidanceAction.BACK_RIGHT: #右輪不動左輪反轉or左輪不動右輪反轉 避障
        stp_left.set_target_speed(TOF_SPEED)
        stp_right.set_target_speed(TOF_SPEED)
        step_to_spin = int((TURN_AROUND_DIST * (TOF_BACK_ANG/180) * PPR) / (WHEEL_DIAM * math.pi))
        if action == AvoidanceAction.BACK_LEFT:
            stp_left.move(-step_to_spin)
        else:
            stp_right.move(-step_to_spin)
    elif action == AvoidanceAction.TURN_LEFT or action == AvoidanceAction.TURN_RIGHT: #左輪不動右輪正轉(車體向左轉)or右輪不動左輪正轉(車體向右轉) 避障
        stp_left.set_target_speed(TOF_SPEED)
        stp_right.set_target_speed(TOF_SPEED)
        step_to_spin = int((TURN_AROUND_DIST * (TOF_TURN_ANG/180) * PPR) / (WHEEL_DIAM * math.pi))
        if action == AvoidanceAction.TURN_LEFT:
            stp_right.move(step_to_spin)
        else:
            stp_left.move(step_to_spin)
    """
    print("Steps: {}, {}; Speed: {}, {}; TargetSpeed: {}, {}".format(
        stp_left.steps_to_go,
        stp_right.steps_to_go,
        stp_left.current_speed,
        stp_right.current_speed,
        stp_left.target_speed,
        stp_right.target_speed,
    ))
    """

def move_control(distance, angual):
    distanceToFollow = distance - MIN_DISTANCE
    stepToFollow = int((distanceToFollow * PPR) / (WHEEL_DIAM * math.pi))
    factor = ctrl_dir(angual)
    try:
        stp_left.set_target_acceleration(ACCELER)
        stp_right.set_target_acceleration(ACCELER)
        if factor['left'] == factor['right']: # Car straight follow
            stp_left.set_target_speed(factor['left'] * MAX_SPEED) #left_wheel setMaxSpeed
            stp_right.set_target_speed(factor['right'] * MAX_SPEED) #right_wheel setMaxSpeed
        elif factor['left'] > factor['right']: # Car right follow
            stp_left.set_target_speed(factor['left'] * MAX_SPEED) #left_wheel setMaxSpeed
            stp_right.set_target_speed(factor['right'] * stp_left.current_speed) #right_wheel setMaxSpeed
        else: # Car left follow
            stp_right.set_target_speed(factor['right'] * MAX_SPEED) #left_wheel setMaxSpeed
            stp_left.set_target_speed(factor['left'] * stp_right.current_speed) #right_wheel setMaxSpeed

    except ZeroDivisionError:
        pass
    stp_left.move(stepToFollow)
    stp_right.move(stepToFollow)
    """
    print("Distance: {}; Steps: {}, {}; Speed: {}, {}; TargetSpeed: {}, {}".format(
        distance,
        stp_left.steps_to_go,
        stp_right.steps_to_go,
        stp_left.current_speed,
        stp_right.current_speed,
        stp_left.target_speed,
        stp_right.target_speed,
    ))
    """

def state_transfer(old_status, distance, angual, bus):
    if old_status == CarStatus.STANDBY:
        ### 偵測是否開始跟隨
        if distance > MAX_DISTANCE:
            return CarStatus.FOLLOWING
        ### 偵測是否跟隨方向
        if distance > STOP_DISTANCE and (angual < -SPIN_ANGUAL or angual > SPIN_ANGUAL):
            stp_left.set_target_acceleration(SPIN_ACCELER)
            stp_right.set_target_acceleration(SPIN_ACCELER)
            return CarStatus.SPINNING
    elif old_status == CarStatus.FOLLOWING:
        ### 偵測是否停下
        if distance <= MIN_DISTANCE:
            stp_left.stop()
            stp_right.stop()
            return CarStatus.STANDBY

        ### 避障部份
        avoid_action = AvoidanceAction.NORMAL
        try:
            if stp_left.steps_to_go != 0 or stp_right.steps_to_go != 0: #左右馬達還在執行
                tofdis_L = read_filtered_distance(bus, TOF_L_I2C_ADDRESS)
                tofdis_FL = read_filtered_distance(bus, TOF_FL_I2C_ADDRESS)
                tofdis_ML = read_filtered_distance(bus, TOF_ML_I2C_ADDRESS)
                tofdis_MR = read_filtered_distance(bus, TOF_MR_I2C_ADDRESS)
                tofdis_FR = read_filtered_distance(bus, TOF_FR_I2C_ADDRESS)
                tofdis_R = read_filtered_distance(bus, TOF_R_I2C_ADDRESS)
                if tuiapp is not None:
                    tuiapp.update_tof(tofdis_L, tofdis_FL, tofdis_ML, tofdis_MR, tofdis_FR, tofdis_R)
                avoid_action = tof10120_judgment(tofdis_L, tofdis_FL, tofdis_ML, tofdis_MR, tofdis_FR, tofdis_R, stp_left.current_speed, stp_right.current_speed)
        except:
            pass

        if avoid_action != AvoidanceAction.NORMAL:
            tof_avoid_control(avoid_action)
            return CarStatus.AVOID
    elif old_status == CarStatus.AVOID:
        ### 確認是否Avoidance完畢
        if stp_left.steps_to_go == 0 and stp_right.steps_to_go == 0: #左右馬達執行完畢
            return CarStatus.FOLLOWING
    elif old_status == CarStatus.SPINNING:
        ### 偵測人是否跑走了
        if distance > MAX_DISTANCE:
            stp_left.set_target_acceleration(ACCELER)
            stp_right.set_target_acceleration(ACCELER)
            return CarStatus.FOLLOWING
        ### 偵測是否跟到了
        if not (distance > STOP_DISTANCE and (angual < -SPIN_ANGUAL or angual > SPIN_ANGUAL)):
            stp_left.set_target_acceleration(ACCELER)
            stp_right.set_target_acceleration(ACCELER)
            stp_left.stop()
            stp_right.stop()
            return CarStatus.STANDBY

    ## 如果狀態沒改變，回傳原來的值
    return old_status

def loop():
    angual = 0
    avg_distance = 0
    car_status = CarStatus.STANDBY
    last_move_time = 0
    move_updated = False

    bus = SMBus(1)
    while (tuiapp is None) or (not tuiapp.stopping):
        try:
            # TODO: Process command queue
            pass
        except:
            pass

        ## 這裡負責狀態的變換以及切換時的指令
        ### 偵測是否沒資料
        if time.time() - last_move_time < MOVE_CMD_EXPIRES:
            car_status = state_transfer(car_status, avg_distance, angual, bus)
        else:
            car_status = state_transfer(car_status, 0, 0, bus)

        ## 這裡負責該狀態的工作
        if car_status == CarStatus.STANDBY:
            pass
        elif car_status == CarStatus.FOLLOWING:
            if move_updated:
                move_control(avg_distance, angual)
        elif car_status == CarStatus.AVOID:
            #左右馬達還在執行
            time.sleep(0.001) #再給他一點時間
        elif car_status == CarStatus.SPINNING:
            car_spin_around(angual, SPIN_SPEED)

def main():
    stp_left.spawn()
    stp_right.spawn()

    try:
        loop()
    except KeyboardInterrupt:
        pass

    stp_left.terminate()
    stp_right.terminate()

if __name__ == '__main__':
    main()
