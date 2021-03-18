from picotui.context import Context
from picotui.defs import *
from picotui.basewidget import Widget
from picotui.screen import Screen
from picotui.widgets import *
from steppyr import StepperController

import os
import sys
import select
import time
from controller import tui_main

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


class TuiApp():
    def __init__(self):
        Screen.attr_color(C_WHITE, None)
        Screen.cls()
        Screen.attr_reset()

        self.stopping = False
        self.kbuf = b""
        self.last_rftime = 0
        self.window = Dialog(0, 0, 48, 20)

        UWBDATA_Y = 1
        self.window.add(2,  UWBDATA_Y, WFrame(44, 4, "UWB Data"))
        self.window.add(12, UWBDATA_Y + 1, WLabel("Distance"))
        self.window.add(30, UWBDATA_Y + 1, WLabel("Angual"))
        ## Value
        self.lbl_uwb_dist = FormatLabel("{:8.1f}", 0.0)
        self.lbl_uwb_ang = FormatLabel("{:>4d}", 0)
        self.window.add(12, UWBDATA_Y + 2, self.lbl_uwb_dist)
        self.window.add(30, UWBDATA_Y + 2, self.lbl_uwb_ang)

        TOFDATA_Y = 6
        self.window.add(2,  TOFDATA_Y, WFrame(44, 4, "TOF Data"))
        self.window.add(5,  TOFDATA_Y + 1, WLabel("   L", 4))
        self.window.add(12, TOFDATA_Y + 1, WLabel("  FL", 4))
        self.window.add(19, TOFDATA_Y + 1, WLabel("  ML", 4))
        self.window.add(26, TOFDATA_Y + 1, WLabel("  MR", 4))
        self.window.add(33, TOFDATA_Y + 1, WLabel("  FR", 4))
        self.window.add(40, TOFDATA_Y + 1, WLabel("   R", 4))
        ## Value
        self.lbl_tof_l  = FormatLabel("{:>4d}", 2000)
        self.lbl_tof_fl = FormatLabel("{:>4d}", 2000)
        self.lbl_tof_ml = FormatLabel("{:>4d}", 2000)
        self.lbl_tof_mr = FormatLabel("{:>4d}", 2000)
        self.lbl_tof_fr = FormatLabel("{:>4d}", 2000)
        self.lbl_tof_r  = FormatLabel("{:>4d}", 2000)
        self.window.add(5,  TOFDATA_Y + 2, self.lbl_tof_l)
        self.window.add(12, TOFDATA_Y + 2, self.lbl_tof_fl)
        self.window.add(19, TOFDATA_Y + 2, self.lbl_tof_ml)
        self.window.add(26, TOFDATA_Y + 2, self.lbl_tof_mr)
        self.window.add(33, TOFDATA_Y + 2, self.lbl_tof_fr)
        self.window.add(40, TOFDATA_Y + 2, self.lbl_tof_r)

        STATUS_Y = 11
        self.window.add(2,  STATUS_Y, WLabel("Status: "))
        self.lbl_status = FormatLabel("{}", "TEST", 32)
        self.window.add(10, STATUS_Y, self.lbl_status)

        STEPPER_Y = 14
        self.window.add(2, STEPPER_Y, WFrame(44, 5, "Steppers"))
        self.window.add(4, STEPPER_Y + 1, WLabel("CurSpeed: "))
        self.window.add(4, STEPPER_Y + 2, WLabel("TgtSpeed: "))
        self.window.add(4, STEPPER_Y + 3, WLabel("StepToGo: "))
        ## Left
        self.lbl_stpl_cs = FormatLabel("{:8.2f}", 0.0)
        self.lbl_stpl_ts = FormatLabel("{:8.2f}", 0.0)
        self.lbl_stpl_tg = FormatLabel("{:8d}", 0)
        self.window.add(20, STEPPER_Y + 1, self.lbl_stpl_cs)
        self.window.add(20, STEPPER_Y + 2, self.lbl_stpl_ts)
        self.window.add(20, STEPPER_Y + 3, self.lbl_stpl_tg)
        ## Right
        self.lbl_stpr_cs = FormatLabel("{:8.2f}", 0.0)
        self.lbl_stpr_ts = FormatLabel("{:8.2f}", 0.0)
        self.lbl_stpr_tg = FormatLabel("{:8d}", 0)
        self.window.add(32, STEPPER_Y + 1, self.lbl_stpr_cs)
        self.window.add(32, STEPPER_Y + 2, self.lbl_stpr_ts)
        self.window.add(32, STEPPER_Y + 3, self.lbl_stpr_tg)

        btn_zzz = WButton(10, "Anywhere")
        self.window.add(8, 19, btn_zzz)

        btn_stop = WButton(8, "Stop")
        self.window.add(32, 19, btn_stop)
        btn_stop.on('click', lambda w: self.stop())

    def stop(self):
        self.stopping = True

    def update_uwb(self, distance, angual):
        self.lbl_uwb_dist.update_value(distance)
        self.lbl_uwb_ang.update_value(angual)

    def update_tof(self, tof_l, tof_fl, tof_ml, tof_mr, tof_fr, tof_r):
        if tof_l != None:
            self.lbl_tof_l.update_value(tof_l)
        if tof_fl != None:
            self.lbl_tof_fl.update_value(tof_fl)
        if tof_ml != None:
            self.lbl_tof_ml.update_value(tof_ml)
        if tof_mr != None:
            self.lbl_tof_mr.update_value(tof_mr)
        if tof_fr != None:
            self.lbl_tof_fr.update_value(tof_fr)
        if tof_r != None:
            self.lbl_tof_r.update_value(tof_r)

    def update_status(self, status):
        self.lbl_status.update_value(status)

    def update_stepper(self, stepper_l: StepperController, stepper_r: StepperController):
        self.lbl_stpl_cs.update_value(stepper_l.current_speed)
        self.lbl_stpl_ts.update_value(stepper_l.target_speed)
        self.lbl_stpl_tg.update_value(stepper_l.steps_to_go)
        self.lbl_stpr_cs.update_value(stepper_r.current_speed)
        self.lbl_stpr_ts.update_value(stepper_r.target_speed)
        self.lbl_stpr_tg.update_value(stepper_r.steps_to_go)

    def get_input(self):
        if self.kbuf:
            key = self.kbuf[0:1]
            self.kbuf = self.kbuf[1:]
        else:
            readable, _, _ = select.select([sys.stdin], [], [], 0)
            if len(readable) == 0:
                return None
            key = os.read(0, 32)
            if key[0] != 0x1b:
                key = key.decode()
                self.kbuf = key[1:].encode()
                key = key[0:1].encode()
        key = KEYMAP.get(key, key)

        if isinstance(key, bytes) and key.startswith(b"\x1b[M") and len(key) == 6:
            if key[3] != 32:
                return None
            row = key[5] - 33
            col = key[4] - 33
            return [col, row]

        return key

    def loop_once(self):
        curtime = time.time()
        if curtime - self.last_rftime < 0.1:
            return False
        elif curtime - self.last_rftime > 5:
            self.window.redraw()
        self.last_rftime = curtime

        key = self.get_input()
        if key is None:
            return False
        res = self.window.handle_input(key)

        if res is not None and res is not True:
            return res

if __name__ == '__main__':
    with Context():
        app = TuiApp()
        """
        while not app.stopping:
            app.loop_once()
            time.sleep(0.01)
        """
        tui_main(app)
