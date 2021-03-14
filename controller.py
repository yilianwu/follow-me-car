from steppyr import StepperController, DIRECTION_CW, DIRECTION_CCW
from steppyr.profiles.accel import AccelProfile
from steppyr.drivers.stepdir import StepDirDriver
from constant import *
import serial, time
import asyncio
from contextlib import suppress
from concurrent.futures import ThreadPoolExecutor
import math
import uwb_data as uwb
from tof10120 import read_filtered_distance
from avoidance import AvoidanceAction, tof10120_judgment
from smbus import SMBus

needWaiting = False #if Running Avoidance?
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

## Helper functions
def car_spin_around(angual, speed=None):
    if speed != None:
        stp_left.set_target_speed(speed)
        stp_right.set_target_speed(speed)
    step_to_spin = int((SPIN_AROUND_DIST * (angual/180) * PPR) / (WHEEL_DIAM * math.pi))
    stp_left.move(-step_to_spin)
    stp_right.move(step_to_spin)

########################

def avg_dist(d):
    global avg_count
    global avg_last
    global avg_sum
    global avg_cur 

    if avg_count >= 5:
        avg_last = avg_sum / 5
        avg_sum = 0
        avg_count = 0
    avg_sum += d
    avg_count += 1

    avg_cur = avg_sum / avg_count
    return ((avg_last + avg_cur) / 2)

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

def angle_filtering(ang):
    global last_angle
    global last_angle_fail

    if last_angle == None:
        last_angle = ang
        return ang
    same_sign = (last_angle * ang) > 0 #bool same_sign 1->同向 0->不同向
    angle_diff = (ang - last_angle)
    if same_sign: #如果兩次讀到的角度方向相同
        if abs(angle_diff) > ANGLE_FILTER_SDIM: #同相位角度差是否大於最大容許值
            retval = last_angle * (1 - ANGLE_FILTER_SFAIL) + ang * ANGLE_FILTER_SFAIL #依比例計算return value
            last_angle_fail += 1 #failed_counter++
        else:
            retval = ang
            last_angle = ang
            last_angle_fail = 0
    else: #如果兩次讀到的角度方向不同
        if abs(angle_diff) > ANGLE_FILTER_NDIM: #不同相位角度差是否大於最大容許值
            retval = last_angle * (1 - ANGLE_FILTER_NFAIL) + ang * ANGLE_FILTER_NFAIL #依比例計算return value
            last_angle_fail += 1
        else:
            retval = ang
            last_angle = ang
            last_angle_fail = 0 #failed_counter++
    
    if last_angle_fail >= ANGLE_FILTER_MAXFAIL: 
        retval = ang
        last_angle = ang

    return retval

def tof_avoid_control(action: AvoidanceAction):
    global needWaiting

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
    
    print("Steps: {}, {}; Speed: {}, {}; TargetSpeed: {}, {}".format(
        stp_left.steps_to_go,
        stp_right.steps_to_go, 
        stp_left.current_speed, 
        stp_right.current_speed, 
        stp_left.target_speed, 
        stp_right.target_speed,    
    ))
    needWaiting = True #Running Avoidance

def uwb_follow_control(distance, angual):
    if distance > STD_DISTANCE:
        distanceToFollow = distance - STD_DISTANCE
        stepToFollow = int((distanceToFollow * PPR) / (WHEEL_DIAM * math.pi))
        factor = ctrl_dir(angual)
        try:
            stp_left.set_target_acceleration(ACCELER)
            stp_right.set_target_acceleration(ACCELER)
            if factor['left'] == factor['right']:
                stp_left.set_target_speed(factor['left'] * MAX_SPEED) #left_wheel setMaxSpeed
                stp_right.set_target_speed(factor['right'] * MAX_SPEED) #right_wheel setMaxSpeed
            elif factor['left'] > factor['right']:
                stp_left.set_target_speed(factor['left'] * MAX_SPEED) #left_wheel setMaxSpeed
                stp_right.set_target_speed(factor['right'] * stp_left.current_speed) #right_wheel setMaxSpeed
            else:
                stp_right.set_target_speed(factor['right'] * MAX_SPEED) #left_wheel setMaxSpeed
                stp_left.set_target_speed(factor['left'] * stp_right.current_speed) #right_wheel setMaxSpeed
        except ZeroDivisionError:
            pass
        stp_left.move(stepToFollow)
        stp_right.move(stepToFollow)
        print("Angual: {}; Distance: {}; Steps: {}, {}; Speed: {}, {}; TargetSpeed: {}, {}".format(
            angual,
            distance,
            stp_left.steps_to_go,
            stp_right.steps_to_go, 
            stp_left.current_speed, 
            stp_right.current_speed, 
            stp_left.target_speed, 
            stp_right.target_speed,    
        ))
    else:
        print("Close enough! TURNING to face to you!")
        if angual < -8 or angual > 8:
            car_spin_around(angual)
        else:
            stp_left.stop()
            stp_right.stop()
            #stop 之後如果設set_current_steps=0

def loop():
    global needWaiting #if Running Avoidance?
    angual = 0
    avg_distance = 0
    uwbdata_updated = False     #表示本次執行是否有讀到uwb資料

    bus = SMBus(1)
    with serial.Serial('/dev/ttyS0', 115200) as ser:
        while True:
            #print("Reading uwb serial", end='')
            try:
                angual, distance = uwb.ser_read(ser)

                #print("angual: {}, distance: {}".format(angual,distance))
                angual = angle_filtering(angual)
                avg_distance = avg_dist(distance)
                uwbdata_updated = True
            except RuntimeError:
                print("invalid UWB value")
            except BlockingIOError:
                pass

            if needWaiting: #確認是否Avoidance完畢
                if stp_left.steps_to_go == 0 and stp_right.steps_to_go == 0: #左右馬達執行完畢
                    needWaiting = False
                    print('Waiting completed')
                else: #左右馬達還在執行
                    time.sleep(0.001) #再給他一點時間 才跳出while迴圈
                    continue

            avoid_action = AvoidanceAction.NORMAL
            try:
                if stp_left.steps_to_go != 0 or stp_right.steps_to_go != 0: #左右馬達還在執行
                    tofdis_L = read_filtered_distance(bus, TOF_L_I2C_ADDRESS)
                    tofdis_ML = read_filtered_distance(bus, TOF_ML_I2C_ADDRESS)
                    tofdis_MR = read_filtered_distance(bus, TOF_MR_I2C_ADDRESS)
                    tofdis_R = read_filtered_distance(bus, TOF_R_I2C_ADDRESS)
                    avoid_action = tof10120_judgment(tofdis_L, tofdis_ML, tofdis_MR, tofdis_R, stp_left.current_speed, stp_right.current_speed)

            except:
                pass

            if avoid_action != AvoidanceAction.NORMAL:
                print(avoid_action)
                tof_avoid_control(avoid_action)
            elif uwbdata_updated:
                uwb_follow_control(avg_distance, angual)
                uwbdata_updated = False
"""
async def stepper_run(stp):
    while True:
        await stp.run()
        await asyncio.sleep(0.0005) #每0.0005秒跑一次 （最多1秒跑(1/0.0005)次）

async def main(): #定義main()為一個協同程序/協程(coroutine)
    evloop = asyncio.get_event_loop() #建立一個Event Loop
    user_task = evloop.run_in_executor(None, loop)
    task1 = asyncio.create_task(stepper_run(stp_left)) # 原本的motorRun()
    task2 = asyncio.create_task(stepper_run(stp_right)) # 原本的motorRun()
    
    await user_task

    task1.cancel()
    task2.cancel()
    with suppress(asyncio.CancelledError): #強制結束task1, task2 忽略exception(Cancel error)
        await task1
        await task2
    stp_left.shutdown()
    stp_right.shutdown()
"""

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
