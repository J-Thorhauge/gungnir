import can
import time

def int32_to_bytes(value):
    """Convert a signed 32-bit integer to a 4-byte little-endian list."""
    return list(value.to_bytes(4, byteorder='little', signed=True))

def send_sdo(bus, tx_id, index, subindex, data, command_specifier):
    """Send an SDO write command over CAN."""
    data_bytes = [command_specifier, index & 0xFF, (index >> 8) & 0xFF, subindex] + data
    data_bytes += [0x00] * (8 - len(data_bytes))  # Pad to 8 bytes
    msg = can.Message(arbitration_id=tx_id, data=data_bytes, is_extended_id=False)
    bus.send(msg)
    time.sleep(0.05)

def enable_motor(bus, tx_id):
    """Enable the motor by sending the standard control word sequence."""
    send_sdo(bus, tx_id, 0x6040, 0x00, [0x06, 0x00], 0x2B)  # Shutdown
    send_sdo(bus, tx_id, 0x6040, 0x00, [0x07, 0x00], 0x2B)  # Switch on
    send_sdo(bus, tx_id, 0x6040, 0x00, [0x0F, 0x00], 0x2B)  # Enable operation

def set_operation_mode(bus, tx_id, mode):
    """Set the operation mode (e.g., 1=position, 3=speed, 4=torque)."""
    send_sdo(bus, tx_id, 0x6060, 0x00, [mode], 0x2F)

def set_target_speed(bus, tx_id, rpm):
    """Set the target speed in RPM."""
    speed_bytes = int32_to_bytes(rpm)
    send_sdo(bus, tx_id, 0x60FF, 0x00, speed_bytes, 0x23)

def fault_reset(bus, tx_id):
    """Send fault reset command."""
    send_sdo(bus, tx_id, 0x6040, 0x00, [0x80, 0x00], 0x2B)
