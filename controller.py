import asyncio
import math
from steppyr import StepperController, DIRECTION_CW, DIRECTION_CCW
from steppyr.profiles.accel import AccelProfile
from steppyr.drivers.stepdir import StepDirDriver
from smbus import SMBus
import RPi.GPIO as GPIO

from constant import *
from tof10120 import read_filtered_distance
from avoidance import AvoidanceAction, tof10120_judgment
from car import CarContext, CarStatus
from server import start_server

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
        return speed_factor, 1.0
    else: #人在右邊->右轉
        return 1.0, speed_factor

def tof_avoid_control(car: CarContext, action: AvoidanceAction):
    car.stop(reset=True)
    car.set_acceleration(TOF_ACCELER)
    if action == AvoidanceAction.SPIN_LEFT or action == AvoidanceAction.SPIN_RIGHT: #向左自轉or向右自轉 避障
        if action == AvoidanceAction.SPIN_LEFT:
            car.spin_around(TOF_SPIN_ANG, TOF_SPEED)
        else:
            car.spin_around(-TOF_SPIN_ANG, TOF_SPEED)
    elif action == AvoidanceAction.BACK_LEFT or action == AvoidanceAction.BACK_RIGHT: #右輪不動左輪反轉or左輪不動右輪反轉 避障
        car.set_speed(TOF_SPEED)
        step_to_spin = int((TURN_AROUND_DIST * (TOF_BACK_ANG/180) * PPR) / (WHEEL_DIAM * math.pi))
        if action == AvoidanceAction.BACK_LEFT:
            car.move_lr(-step_to_spin, 0)
        else:
            car.move_lr(0, -step_to_spin)
    elif action == AvoidanceAction.TURN_LEFT or action == AvoidanceAction.TURN_RIGHT: #左輪不動右輪正轉(車體向左轉)or右輪不動左輪正轉(車體向右轉) 避障
        car.set_speed(TOF_SPEED)
        step_to_spin = int((TURN_AROUND_DIST * (TOF_TURN_ANG/180) * PPR) / (WHEEL_DIAM * math.pi))
        if action == AvoidanceAction.TURN_LEFT:
            car.move_lr(0, step_to_spin)
        else:
            car.move_lr(step_to_spin, 0)

def move_control(car: CarContext):
    if not car.move_needupdate:
        return

    distanceToFollow = car.move_distance - MIN_DISTANCE
    stepToFollow = int((distanceToFollow * PPR) / (WHEEL_DIAM * math.pi))
    factor_left, factor_right = ctrl_dir(car.move_angual)
    try:
        car.set_acceleration_lr(factor_left * ACCELER, factor_right * ACCELER)
        car.set_speed_lr(factor_left * MAX_SPEED, factor_right * MAX_SPEED)
        car.move_lr(factor_left * stepToFollow, factor_right * stepToFollow)
        car.move_resetupdate()
    except ZeroDivisionError:
        pass

def state_transfer(car: CarContext, bus: SMBus):
    distance = car.move_distance
    angual = car.move_angual
    old_status = car.status

    if old_status == CarStatus.STANDBY:
        ### 偵測是否開始跟隨
        if distance > MAX_DISTANCE:
            return CarStatus.FOLLOWING
        ### 偵測是否跟隨方向
        if distance > STOP_DISTANCE and (angual < -SPIN_ANGUAL or angual > SPIN_ANGUAL):
            car.set_acceleration(SPIN_ACCELER)
            return CarStatus.SPINNING
    elif old_status == CarStatus.FOLLOWING:
        ### 偵測是否停下
        if distance <= MIN_DISTANCE:
            car.stop()
            return CarStatus.STANDBY

        ### 避障部份
        avoid_action = AvoidanceAction.NORMAL
        try:
            if car.has_steps_to_go(): #左右馬達還在執行
                tofdis_L = read_filtered_distance(bus, TOF_L_I2C_ADDRESS)
                tofdis_FL = read_filtered_distance(bus, TOF_FL_I2C_ADDRESS)
                tofdis_ML = read_filtered_distance(bus, TOF_ML_I2C_ADDRESS)
                tofdis_MR = read_filtered_distance(bus, TOF_MR_I2C_ADDRESS)
                tofdis_FR = read_filtered_distance(bus, TOF_FR_I2C_ADDRESS)
                tofdis_R = read_filtered_distance(bus, TOF_R_I2C_ADDRESS)
                avoid_action = tof10120_judgment(tofdis_L, tofdis_FL, tofdis_ML, tofdis_MR, tofdis_FR, tofdis_R, car.stp_left.current_speed, car.stp_right.current_speed)
        except:
            pass

        if avoid_action != AvoidanceAction.NORMAL:
            tof_avoid_control(car, avoid_action)
            return CarStatus.AVOID
    elif old_status == CarStatus.AVOID:
        ### 確認是否Avoidance完畢
        if not car.has_steps_to_go(): #左右馬達執行完畢
            return CarStatus.FOLLOWING
    elif old_status == CarStatus.SPINNING:
        ### 偵測人是否跑走了
        if distance > MAX_DISTANCE:
            car.set_acceleration(ACCELER)
            return CarStatus.FOLLOWING
        ### 偵測是否跟到了
        if not (distance > STOP_DISTANCE and (angual < -SPIN_ANGUAL or angual > SPIN_ANGUAL)):
            car.set_acceleration(ACCELER)
            car.stop()
            return CarStatus.STANDBY

    ## 如果狀態沒改變，回傳原來的值
    return old_status

async def loop(car: CarContext):
    bus = SMBus(1)
    while True:
        try:
            # TODO: Process command queue
            pass
        except:
            pass

        ## 這裡負責狀態的變換以及切換時的指令
        car.status = state_transfer(car, bus)

        ## 這裡負責該狀態的工作
        if car.status == CarStatus.STANDBY:
            await asyncio.sleep(0)
        elif car.status == CarStatus.FOLLOWING:
            move_control(car)
        elif car.status == CarStatus.AVOID:
            #左右馬達還在執行
            await asyncio.sleep(0.001) #再給他一點時間
        elif car.status == CarStatus.SPINNING:
            car.spin_around(car.move_angual, SPIN_SPEED)
        await asyncio.sleep(0)

async def main():
    car = CarContext(StepDirDriver(6, 5), StepDirDriver(24, 23))
    car.set_acceleration(ACCELER)
    car.set_speed(MAX_SPEED)
    car.activate()

    await start_server(car)

    try:
        await loop(car)
    except KeyboardInterrupt:
        pass

    car.deactivate()

if __name__ == '__main__':
    asyncio.run(main())
