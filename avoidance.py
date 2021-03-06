from enum import Enum
from utils import vmap
from constant import MAX_SPEED
class AvoidanceAction(Enum):
    SPIN_LEFT = -3
    BACK_LEFT = -2
    TURN_LEFT = -1
    NORMAL = 0
    TURN_RIGHT = 1
    BACK_RIGHT = 2
    SPIN_RIGHT = 3

avoid_alertcnt = 3
avoid_alert = False

def tof10120_judgment(dis_L, dis_FL, dis_ML, dis_MR, dis_FR, dis_R, left_speed, right_speed):
    global avoid_alert
    global avoid_alertcnt
    """
    print('TOF: {}, {}, {}, {}'.format(dis_L, dis_ML, dis_MR, dis_R))
    """
    car_speed = max(left_speed,right_speed)
    dis_LR_diff = (dis_L +dis_FL + dis_ML) - (dis_MR + dis_FR + dis_R)

    basic_thld = vmap(car_speed, 0, MAX_SPEED, 600, 750)
    barrier_thld = vmap(car_speed, 0, MAX_SPEED, 600, 750) 
    barrier_thld_m = vmap(car_speed, 0, MAX_SPEED, 700, 950)

    back_thld = vmap(car_speed, 0, MAX_SPEED, 650, 800)
    back_thld_m = vmap(car_speed, 0, MAX_SPEED, 600, 700)
    turn_thld = vmap(car_speed, 0, MAX_SPEED, 700, 900)
    turn_thld_m = vmap(car_speed, 0, MAX_SPEED, 900, 1150)

    # 如果偵測到任一TOF < basic_thld
    if dis_L < basic_thld or dis_FL < basic_thld or dis_ML < basic_thld or dis_MR < basic_thld or dis_FR < basic_thld or dis_R < basic_thld:
        if avoid_alertcnt > 0:
            avoid_alertcnt -= 1
    else:
        if avoid_alertcnt < 4:
            avoid_alertcnt += 1
    
    if avoid_alertcnt <= 0:
        avoid_alert = True
    elif avoid_alertcnt >= 3:
        avoid_alert = False

    if avoid_alert:
        spin_barrier = 0
        if dis_L < barrier_thld:
            spin_barrier += 1
        if dis_ML < barrier_thld_m:
            spin_barrier += 1
        if dis_MR < barrier_thld_m:
            spin_barrier += 1
        if dis_R < barrier_thld:
            spin_barrier += 1

        if spin_barrier >= 6:
            # 四周都有障礙，需要原地旋轉
            if dis_LR_diff < 0: #左邊有障礙物
                return AvoidanceAction.SPIN_RIGHT
            else: #右邊有障礙物
                return AvoidanceAction.SPIN_LEFT
        elif dis_L < back_thld or dis_FL < back_thld or dis_ML < back_thld_m or dis_MR < back_thld_m or dis_FR < back_thld or dis_R < back_thld:
            if dis_LR_diff < 0: #左邊有障礙物
                return AvoidanceAction.BACK_RIGHT
            else: #右邊有障礙物
                return AvoidanceAction.BACK_LEFT
        elif dis_L < turn_thld or dis_FL < turn_thld or dis_ML < turn_thld_m or dis_MR < turn_thld_m or dis_FR<turn_thld or dis_R < turn_thld:
            if dis_LR_diff < 0: #左邊有障礙物
                return AvoidanceAction.TURN_RIGHT
            else: #右邊有障礙物
                return AvoidanceAction.TURN_LEFT

    return AvoidanceAction.NORMAL

