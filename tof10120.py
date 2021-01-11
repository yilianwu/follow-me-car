from smbus import SMBus

REGADDR_DIS_RAW = 0x00
REGADDR_DIS_FILTER = 0x04
REGADDR_DIS_OFFSET = 0x06
REGADDR_CFG_DATAMODE = 0x08
REGADDR_CFG_SENDMODE = 0x09
REGADDR_ATTR_MAXDIS = 0x0c
REGADDR_CFG_DEVADDR = 0x0f

def read_raw_distance(bus: SMBus, devaddr):
    data = bus.read_i2c_block_data(devaddr, REGADDR_DIS_RAW, 2)
    return int.from_bytes(data, 'big')

def read_filtered_distance(bus: SMBus, devaddr):
    data = bus.read_i2c_block_data(devaddr, REGADDR_DIS_FILTER, 2)
    return int.from_bytes(data, 'big')

def read_distance_offset(bus: SMBus, devaddr):
    data = bus.read_i2c_block_data(devaddr, REGADDR_DIS_OFFSET, 2)
    return int.from_bytes(data, 'big', signed=True)

def write_distance_offset(bus: SMBus, devaddr, value):
    data = list(value.to_bytes(2, 'big', signed=True))
    bus.write_i2c_block_data(devaddr, REGADDR_DIS_OFFSET, data)
