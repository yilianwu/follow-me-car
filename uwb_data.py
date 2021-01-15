import serial, time
import sys
from select import select
import tty
import termios

cur_move = 0

FAKE_DATA = [
    (0, 0),
    (0, 150),
    (30, 120),
    (-30, 120),
]

def ser_read(ser):
    global cur_move
    timeout = 0.01
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        ch = ''
        try:
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        if ch == 's':
            cur_move = 0
        elif ch == 'w':
            cur_move = 1
        elif ch == 'a':
            cur_move = 2
        elif ch == 'd':
            cur_move = 3

    return FAKE_DATA[cur_move]
