from enum import Enum

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

def tof10120_judgment(dis_L, dis_ML, dis_MR, dis_R):
    global avoid_alert
    global avoid_alertcnt
    print('TOF: {}, {}, {}, {}'.format(dis_L, dis_ML, dis_MR, dis_R))

    dis_LR_diff = (dis_L + dis_ML) - (dis_MR + dis_R)

    if dis_L < 900 or dis_ML < 900 or dis_MR < 900 or dis_R < 900: #如果任一tof偵測到距離<200mm
        if avoid_alertcnt > 0:
            avoid_alertcnt -= 1
    else:
        if avoid_alertcnt < 3:
            avoid_alertcnt += 1
    
    if avoid_alertcnt <= 0:
        avoid_alert = True
    elif avoid_alertcnt >= 2:
        avoid_alert = False

    if avoid_alert:
        spin_barrier = 0
        if dis_L < 500:
            spin_barrier += 1
        if dis_ML < 700:
            spin_barrier += 1
        if dis_MR < 700:
            spin_barrier += 1
        if dis_R < 500:
            spin_barrier += 1

        if spin_barrier >= 4:
            # 四周都有障礙，需要原地旋轉
            if dis_LR_diff < 0: #左邊有障礙物
                return AvoidanceAction.SPIN_RIGHT
            else: #右邊有障礙物
                return AvoidanceAction.SPIN_LEFT
        elif dis_L < 450 or dis_ML < 350 or dis_MR < 350 or dis_R < 450:
            if dis_LR_diff < 0: #左邊有障礙物
                return AvoidanceAction.BACK_RIGHT
            else: #右邊有障礙物
                return AvoidanceAction.BACK_LEFT
        elif dis_L < 700 or dis_ML < 900 or dis_MR < 900 or dis_R < 700:
            if dis_LR_diff < 0: #左邊有障礙物
                return AvoidanceAction.TURN_RIGHT
            else: #右邊有障礙物
                return AvoidanceAction.TURN_LEFT

    return AvoidanceAction.NORMAL

