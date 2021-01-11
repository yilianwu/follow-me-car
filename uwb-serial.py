import serial, time
with serial.Serial('/dev/ttyS0', 115200) as ser:
    while True:
        while True:
            head_byte=ser.read()
            if head_byte == b"\x2a":
                break
        len_byte=ser.read()
        data=ser.read(int(len_byte[0]))
        uwb_angual=int.from_bytes(data[3:7], 'little', signed=True)
        uwb_distance=int.from_bytes(data[7:11], 'little', signed=True)
        print("angual:",uwb_angual)
        print("distance:",uwb_distance)