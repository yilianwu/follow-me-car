from picotui.context import Context
from picotui.defs import *
from picotui.basewidget import Widget
from picotui.screen import Screen
from picotui.widgets import *
from datetime import datetime

window = None

class FormatLabel(Widget):
    def __init__(self, fmttext, val, w=0):
        self.f = fmttext
        self.v = val
        self.t = self.f.format(self.v)
        self.h = 1
        self.w = w
        if not w:
            self.auto_w = True
            self.w = len(self.t)
        else:
            self.auto_w = False

    def update_value(self, val):
        self.v = val
        self.t = self.f.format(self.v)
        if self.auto_w:
            self.w = len(self.t)
        self.redraw()

    def redraw(self):
        self.goto(self.x, self.y)
        self.wr_fixedw(self.t, self.w)


def init_tui():
    global window

    Screen.attr_color(C_WHITE, None)
    Screen.cls()
    Screen.attr_reset()

    window = Dialog(0, 0, 48, 20)

    UWBDATA_Y = 1
    window.add(2,  UWBDATA_Y, WFrame(44, 4, "UWB Data"))
    window.add(12,  UWBDATA_Y + 1, WLabel("Distance"))
    window.add(30, UWBDATA_Y + 1, WLabel("Angual"))
    ## Value
    window.add(12,  UWBDATA_Y + 2, FormatLabel("{:.1f}", 0.0))
    window.add(30, UWBDATA_Y + 2, FormatLabel("{}", 0))

    TOFDATA_Y = 6
    window.add(2,  TOFDATA_Y, WFrame(44, 4, "TOF Data"))
    window.add(5,  TOFDATA_Y + 1, WLabel("   L", 4))
    window.add(12, TOFDATA_Y + 1, WLabel("  FL", 4))
    window.add(19, TOFDATA_Y + 1, WLabel("  ML", 4))
    window.add(26, TOFDATA_Y + 1, WLabel("  MR", 4))
    window.add(33, TOFDATA_Y + 1, WLabel("  FR", 4))
    window.add(40, TOFDATA_Y + 1, WLabel("   R", 4))
    ## Value
    window.add(5,  TOFDATA_Y + 2, FormatLabel("{:>4d}", 2000))
    window.add(12, TOFDATA_Y + 2, FormatLabel("{:>4d}", 2000))
    window.add(19, TOFDATA_Y + 2, FormatLabel("{:>4d}", 2000))
    window.add(26, TOFDATA_Y + 2, FormatLabel("{:>4d}", 2000))
    window.add(33, TOFDATA_Y + 2, FormatLabel("{:>4d}", 2000))
    window.add(40, TOFDATA_Y + 2, FormatLabel("{:>4d}", 2000))

    STATUS_Y = 11
    window.add(2, STATUS_Y, WLabel("Status: "))
    window.add(10, STATUS_Y, WLabel("TEST"))

    STEPPER_Y = 14
    window.add(2, STEPPER_Y, WFrame(44, 5, "Steppers"))
    window.add(4, STEPPER_Y + 1, WLabel("CurSpeed: "))
    window.add(4, STEPPER_Y + 2, WLabel("TgtSpeed: "))
    window.add(4, STEPPER_Y + 3, WLabel("StepToGo: "))
    ## Left
    window.add(20, STEPPER_Y + 1, FormatLabel("{:.2f}", 0.0))
    window.add(20, STEPPER_Y + 2, FormatLabel("{:.2f}", 0.0))
    window.add(20, STEPPER_Y + 3, FormatLabel("{:d}", 0))
    ## Right
    window.add(32, STEPPER_Y + 1, FormatLabel("{:.2f}", 0.0))
    window.add(32, STEPPER_Y + 2, FormatLabel("{:.2f}", 0.0))
    window.add(32, STEPPER_Y + 3, FormatLabel("{:d}", 0))

    btn_stop = WButton(8, "Stop")
    window.add(32, 19, btn_stop)
    btn_stop.finish_dialog = ACTION_CANCEL
    #btn_stop.on('click', )

if __name__ == '__main__':
    with Context():
        init_tui()
        window.loop()
