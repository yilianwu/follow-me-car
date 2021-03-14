import math
# TOF_L_I2C_ADDRESS = 0x53
# TOF_FL_I2C_ADDRESS = 0x54
# TOF_ML_I2C_ADDRESS = 0x55
# TOF_MR_I2C_ADDRESS = 0x56
# TOF_FR_I2C_ADDRESS = 0x57
# TOF_R_I2C_ADDRESS = 0x58

TOF_L_I2C_ADDRESS = 0x53
TOF_ML_I2C_ADDRESS = 0x54
TOF_MR_I2C_ADDRESS = 0x55
TOF_R_I2C_ADDRESS = 0x56
TOF_SPIN_ANG = 45
TOF_BACK_ANG = 30
TOF_TURN_ANG = 30

WHEEL_DIAM = 12.5
CAR_TREAD = 33.0
SPIN_AROUND_DIST = (CAR_TREAD * math.pi) / 2
TURN_AROUND_DIST = (CAR_TREAD * 2 * math.pi) / 2
PPR = 800 # pulse/rev
RPM = 220
STD_DISTANCE = 70
MAX_SPEED = (RPM * PPR) / 60
ACCELER = 800
TOF_SPEED = MAX_SPEED
TOF_ACCELER = 1600

ANGLE_FILTER_SDIM = 20
ANGLE_FILTER_NDIM = 5
ANGLE_FILTER_SFAIL = 0.2 #計算角度的比例參數 last_angle * (1 - ANGLE_FILTER_SFAIL) + ang * ANGLE_FILTER_SFAIL
ANGLE_FILTER_NFAIL = 0.05 #計算角度的比例參數 last_angle * (1 - ANGLE_FILTER_NFAIL) + ang * ANGLE_FILTER_NFAIL
ANGLE_FILTER_MAXFAIL = 50 #

TURNING_ANGLE_PN80 = 0.02
TURNING_ANGLE_PN70 = 0.05
TURNING_ANGLE_PN60 = 0.15
TURNING_ANGLE_PN50 = 0.3
TURNING_ANGLE_PN40 = 0.5
TURNING_ANGLE_PN30 = 0.75
TURNING_ANGLE_PN20 = 0.8
TURNING_ANGLE_PN8 = 0.95