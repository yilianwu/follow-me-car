import time

from steppyr import StepperController, DIRECTION_CW, DIRECTION_CCW
from steppyr.profiles.accel import AccelProfile
from steppyr.drivers import Driver

from constant import *

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

class CarContext:
    def __init__(self, left_driver: Driver, right_driver: Driver):
        self.stp_left = StepperController(
            left_driver,
            AccelProfile(),
        )
        self.stp_right = StepperController(
            right_driver,
            AccelProfile(),
        )
        self.status = CarStatus.STANDBY
        # Moving target
        self.__move_angual = 0
        self.__move_distance = 0
        self.__move_lasttime = 0
        self.__move_needupdate = False

    def activate(self):
        self.stp_left.spawn()
        self.stp_right.spawn()
        self.stp_left.activate()
        self.stp_right.activate()

    def deactivate(self):
        self.stp_left.shutdown()
        self.stp_right.shutdown()
        self.stp_left.terminate()
        self.stp_right.terminate()

    ## Status
    def has_steps_to_go(self):
        return self.stp_left.steps_to_go > 0 or self.stp_right.steps_to_go > 0

    ## Configurations
    def set_acceleration(self, accel):
        self.stp_left.set_target_acceleration(accel)
        self.stp_right.set_target_acceleration(accel)

    def set_acceleration(self, left, right):
        self.stp_left.set_target_acceleration(left)
        self.stp_right.set_target_acceleration(right)

    def set_speed(self, speed):
        self.stp_left.set_target_speed(speed)
        self.stp_right.set_target_speed(speed)

    def set_speed(self, left, right):
        self.stp_left.set_target_speed(left)
        self.stp_right.set_target_speed(right)

    ## Actions
    def move(self, steps):
        self.stp_left.move(steps)
        self.stp_right.move(steps)

    def move(self, left, right):
        self.stp_left.move(left)
        self.stp_right.move(right)

    def stop(self, reset=True):
        self.stp_left.stop()
        self.stp_right.stop()
        if reset:
            self.stp_left.set_current_steps(0)
            self.stp_right.set_current_steps(0)

    def spin_around(self, angual, speed=None):
        if speed != None:
            self.stp_left.set_target_speed(speed)
            self.stp_right.set_target_speed(speed)
        step_to_spin = int((SPIN_AROUND_DIST * (angual * SPIN_BUFFER / 180) * PPR) / (WHEEL_DIAM * math.pi))
        self.stp_left.move(-step_to_spin)
        self.stp_right.move(step_to_spin)

    ## Move target
    @property
    def move_angual(self):
        ### 偵測是否沒資料
        if MOVE_CMD_EXPIRES == None or (time.time() - self.__move_lasttime) < MOVE_CMD_EXPIRES:
            return self.__move_angual
        else:
            return 0

    @move_angual.setter
    def move_angual(self, value):
        self.__move_angual = value
        self.__move_lasttime = time.time()
        self.__move_needupdate = True

    @property
    def move_distance(self):
        ### 偵測是否沒資料
        if MOVE_CMD_EXPIRES == None or (time.time() - self.__move_lasttime) < MOVE_CMD_EXPIRES:
            return self.__move_distance
        else:
            return 0

    @move_distance.setter
    def move_distance(self, value):
        self.__move_distance = value
        self.__move_lasttime = time.time()
        self.__move_needupdate = True

    @property
    def move_needupdate(self):
        return self.__move_needupdate

    def move_resetupdate(self):
        self.__move_needupdate = False
