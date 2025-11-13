import can
import time

class Motor:
    def __init__(self, motor_name, motor_id, motor_type):
        """Initialize the motor with an ID and type.
        Args:
            motor_name (str): Convinient human-friendly name.
            motor_id (int): Unique CAN id for the motor.
            motor_type (str): Type of the motor 'small' for the coolass steppers or 'big' for MyActuator Motors.
        """
        self.motor_name = motor_name
        self.motor_id = motor_id # CAN address
        self.tx_id = 0x600 + motor_id
        self.rx_id = 0x580 + motor_id
        self.motor_type = motor_type

    def send_sdo(self, bus, index, subindex, data, command_specifier):
        """Send an SDO write command over CAN."""
        data_bytes = [command_specifier, index & 0xFF, (index >> 8) & 0xFF, subindex] + data
        data_bytes += [0x00] * (8 - len(data_bytes))  # Pad to 8 bytes
        msg = can.Message(arbitration_id=self.tx_id, data=data_bytes, is_extended_id=False)
        bus.send(msg)
        time.sleep(0.05)
    
    def int32_to_bytes(self, value):
        """Convert a signed 32-bit integer to a 4-byte little-endian list."""
        return list(value.to_bytes(4, byteorder='little', signed=True))

    def enable_motor(self, bus):
        """Enable motor"""
        self.send_sdo(bus, 0x6040, 0x00, [0x06, 0x00], 0x2B)  # Shutdown
        self.send_sdo(bus, 0x6040, 0x00, [0x07, 0x00], 0x2B)  # Switch on
        self.send_sdo(bus, 0x6040, 0x00, [0x0F, 0x00], 0x2B)  # Enable operation
        print(f"{self.motor_name} enabled.")

    def disable(self, bus):
        """Disable motor"""
        self.send_sdo(bus, 0x6040, 0x00, [0x06, 0x00], 0x2B)  # Shutdown
    
    def set_operation_mode(self, bus, mode):
        """Set the operation mode (e.g., 1=position, 3=speed, 4=torque)."""
        self.send_sdo(bus, 0x6060, 0x00, [mode], 0x2F)
        print(f"{self.motor_name} set to operation mode {mode}.")

    def set_target_speed(self, bus, rpm):
        """Set the target speed in RPM."""
        speed_bytes = self.int32_to_bytes(rpm)
        self.send_sdo(bus, 0x60FF, 0x00, speed_bytes, 0x23)
        print(f"{self.motor_name} target speed set to {rpm} RPM.")

    def fault_reset(self, bus):
        """Send fault reset command."""
        self.send_sdo(bus, 0x6040, 0x00, [0x80, 0x00], 0x2B)

    