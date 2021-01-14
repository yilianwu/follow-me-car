import serial, time

def ser_read(ser):
    ser.timeout = 0.01
    while True:
        #print('.', end='', flush=True)
        head_byte=ser.read()
        if len(head_byte) < 1:
            raise BlockingIOError
        if head_byte == b'\x2a':
            break #取消迴圈
    #print('#', end='', flush=True)
    ser.timeout = None
    len_byte=ser.read() #len_byte=19
    data=ser.read(int(len_byte[0])) 
    #print('$', end='', flush=True)
    uwb_angual=int.from_bytes(data[3:7], 'little', signed=True)
    uwb_distance=int.from_bytes(data[7:11], 'little', signed=True)
    check = ser.read()
    if ord(check) != uwb_checksum(data):
        raise RuntimeError("Invalid UWB checksum")
    foot = ser.read()
    if foot != b'\x23':
        raise RuntimeError("Invalid foot")
    return uwb_angual, uwb_distance

def uwb_checksum(data):
    checksum = 0
    for b in data:
        checksum ^= b #XOR
    return checksum