from smbus import SMBus

REGADDR_DIS_RAW = 0x00 # 0x00-0x01 實時距離 (ReadOnly)
REGADDR_DIS_FILTER = 0x04 #0x04-0x05 濾波距離 (ReadOnly)
REGADDR_DIS_OFFSET = 0x06 #0x06-0x07 距離偏差 (ReadWrite)
REGADDR_CFG_DATAMODE = 0x08 #距離數據模式 (ReadWrite)
REGADDR_CFG_SENDMODE = 0x09 #距離發送模式 (ReadWrite)
REGADDR_ATTR_MAXDIS = 0x0c #0x0c-0x0d 最大測量距離 (ReadOnly)
REGADDR_CFG_DEVADDR = 0x0f #I2C Device Address (ReadWrite)

def read_raw_distance(bus: SMBus, devaddr): #讀取實時距離
    data = bus.read_i2c_block_data(devaddr, REGADDR_DIS_RAW, 2)
    return int.from_bytes(data, 'big') #把bytes轉換成decimal

def read_filtered_distance(bus: SMBus, devaddr): #讀取濾波距離
    data = bus.read_i2c_block_data(devaddr, REGADDR_DIS_FILTER, 2)
    return int.from_bytes(data, 'big')

def read_distance_offset(bus: SMBus, devaddr): #讀取距離偏差
    data = bus.read_i2c_block_data(devaddr, REGADDR_DIS_OFFSET, 2)
    return int.from_bytes(data, 'big', signed=True)

def write_distance_offset(bus: SMBus, devaddr, value): #寫入距離偏差
    data = list(value.to_bytes(2, 'big', signed=True))
    bus.write_i2c_block_data(devaddr, REGADDR_DIS_OFFSET, data)
