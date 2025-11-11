import can
import time
from canopen_helpers import enable_motor, set_operation_mode, set_target_speed, fault_reset

# Setup CAN interface (adjust channel and bitrate as needed)
bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=500000)

NODE_ID1 = 0x68  # 104 in decimal
NODE_ID2 = 0x65
tx_id1 = 0x600 + NODE_ID1  # 0x668
tx_id2 = 0x600 + NODE_ID2  # 0x668
rx_id1 = 0x580 + NODE_ID1  # 0x5E8
rx_id2 = 0x580 + NODE_ID2  # 0x5E8


# Reset any faults
fault_reset(bus, tx_id1)

# Enable motor
enable_motor(bus, tx_id1)

# Set to speed mode (3)
set_operation_mode(bus, tx_id1, 3)

# Set speed to 4000 RPM
set_target_speed(bus, tx_id1, 0)


# Reset any faults
fault_reset(bus, tx_id2)

# Enable motor
enable_motor(bus, tx_id2)

# Set to speed mode (3)
set_operation_mode(bus, tx_id2, 3)

# Set speed to 4000 RPM
set_target_speed(bus, tx_id2, 0)

time.sleep(8)
while True:

    set_target_speed(bus, tx_id1, 4000)


    set_target_speed(bus, tx_id2, 0)

    time.sleep(2)

    set_target_speed(bus, tx_id1, 0)

    set_target_speed(bus, tx_id2, 4000)

    time.sleep(2)

