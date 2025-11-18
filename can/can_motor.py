import can
import time

class Motor:
    def __init__(self, motor_name, motor_id, motor_type, acc = 100, dec = 100):
        """Initialize the motor with an ID and type.
        Args:
            motor_name (str): Convinient human-friendly name. Eg. Joint 1
            motor_id (int): Unique CAN id for the motor. Convention for Gungir is 101 for joint 1, 102 for joint 2, etc.
            motor_type (str): Type of the motor 'small' for the coolass steppers or 'big' for MyActuator Motors.
            Then it sets up the CAN tx and rx ids according to the datasheet.
            Acceleration and deceleration are in units of rps/s.
        """
        self.motor_name = motor_name
        self.motor_id = motor_id # CAN address
        self.tx_id = 0x600 + motor_id
        self.rx_id = 0x580 + motor_id
        self.motor_type = motor_type
        self.acc = acc
        self.dec = dec
        self.set_mode = 10 # mode value placeholder, 10 is an invalid mode.
        self.rx_buffer = []  # Buffer for incoming CAN messages
        self.speed = 0
        self.position = 0

    def send_sdo(self, bus, index, subindex, data, command_specifier):
        """Send an SDO write command over CAN.
        Here is the structure of an SDO write message: tak til ascii table generator
        +======================+==========================+=============+=========+===========+===================+
        | Example instruction  | Node id 0x600 + motor_id | Data length |  Index  | Sub-index |    Data bytes     |
        +======================+==========================+=============+====+====+===========+====+====+====+====+
        | Set speed to 100 RPM |            601           |     23h     | FF | 60 |     00    | 64 | 00 | 00 | 00 |
        +----------------------+--------------------------+-------------+----+----+-----------+----+----+----+----+
        Note: Data bytes and index are in little-endian format. FF60 = 0x60FF and 64 00 00 00 = 00 00 00 64 = 100.
        """
        data_bytes = [command_specifier, index & 0xFF, (index >> 8) & 0xFF, subindex] + data
        data_bytes += [0x00] * (8 - len(data_bytes))  # Pad to 8 bytes
        msg = can.Message(arbitration_id=self.tx_id, data=data_bytes, is_extended_id=False)
        bus.send(msg)
        time.sleep(0.0) # vi skal lige finde ud af hvor lang tid man skal vente egentlig

    def receive_sdo(self, bus):
        """Receive a response from the motor using the rx buffer. /maybe/ kig på om timeout er nødvendigt"""
        try:
            print("Listening for CAN messages on")
            while True:
                # Read a message from the CAN bus
                message = bus.recv()

                if message is not None:
                        print(f"Received: {message}")

                if message and message.arbitration_id == self.rx_id:

                    if message is not None:
                        print(f"Received: {message} lol")

        except can.CanError as e:
            print(f"CAN error: {e}")
    
    def int32_to_bytes(self, value):
        """Convert a signed 32-bit integer to a 4-byte little-endian list."""
        return list(value.to_bytes(4, byteorder='little', signed=True))

    def enable_motor(self, bus):
        """Enable motor
        And set acceleration and deceleration to reasonable values.
        Note: Acceleration and deceleration are in units of 0.1 rpm/s. so for 10 rpm/s, send 100 (0x64).     
        """
        self.send_sdo(bus, 0x6040, 0x00, [0x06, 0x00], 0x2B)  # Shutdown
        self.send_sdo(bus, 0x6040, 0x00, [0x07, 0x00], 0x2B)  # Switch on
        self.send_sdo(bus, 0x6040, 0x00, [0x0F, 0x00], 0x2B)  # Enable operation
        print(f"{self.motor_name} enabled.")
        #Set acceleration and deceleration note til os, det forskelligt hvornår de sætter deres vals i forhold til mode
        acc_bytes = self.int32_to_bytes(self.acc*10)
        dec_bytes = self.int32_to_bytes(self.dec*10)
        self.send_sdo(bus, 0x6083, 0x00, acc_bytes, 0x23)  # Acceleration
        self.send_sdo(bus, 0x6084, 0x00, dec_bytes, 0x23)  # Deceleration

    def disable(self, bus):
        """Disable motor"""
        self.send_sdo(bus, 0x6040, 0x00, [0x06, 0x00], 0x2B)  # Shutdown
    
    def set_operation_mode(self, bus, mode):
        """Set the operation mode (e.g., 1=position, 3=speed, 4=torque)."""
        self.send_sdo(bus, 0x6060, 0x00, [mode], 0x2F)
        self.set_mode = mode
        print(f"{self.motor_name} set to operation mode {mode}.")

    def set_target_speed(self, bus, rpm):
        """Set the target speed in RPM."""
        if self.set_mode != 3:
            print(f"Warning: {self.motor_name} is not in speed mode. Current mode: {self.set_mode}")
            return
        speed_bytes = self.int32_to_bytes(rpm)
        self.send_sdo(bus, 0x60FF, 0x00, speed_bytes, 0x23)
        print(f"{self.motor_name} target speed set to {rpm} RPM.")
    
    def set_target_position(self, bus, position, speed = 2000):
        """Set the target position in pulse? counts. Speed in RPM.
        """
        if self.set_mode != 1:
            print(f"Warning: {self.motor_name} is not in position mode. Current mode: {self.set_mode}")
            return
        speed_bytes = self.int32_to_bytes(speed)
        position_bytes = self.int32_to_bytes(position)
        self.send_sdo(bus, 0x6081, 0x00, speed_bytes, 0x23)  # Set speed for position movement
        self.send_sdo(bus, 0x607A, 0x00, position_bytes, 0x23)  # Set target position
        self.send_sdo(bus, 0x6040, 0x00, [0x0F, 0x00], 0x2B)  # Start movement
        self.send_sdo(bus, 0x6040, 0x00, [0x1F, 0x00], 0x2B)  
        print(f"{self.motor_name} target position set to {position} counts at speed {speed} RPM.")

    def stop_motor(self, bus):
        """Stop the motor smoothly with 605Ah using 6085h (1000 rps/s)deceleration."""
        self.send_sdo(bus, 0x6040, 0x00, [0x02, 0x00], 0x2B)
        print(f"{self.motor_name} stopping motor.")
    
    def read_position(self, bus):
        """Read the current position from the motor."""
        self.send_sdo(bus, 0x6064, 0x00, [], 0x40)
        """ response = self.receive_sdo(bus)
        if response:
            position = int.from_bytes(response[4:8], byteorder='little', signed=True)
            print(f"{self.motor_name} current position: {position} counts/pulses.")
            return position
        else:
            print(f"Failed to read position from {self.motor_name}.")
            return None """
        
    def read_speed(self, bus):
        """Read the current speed from the motor."""
        self.send_sdo(bus, 0x606C, 0x00, [], 0x40)
        """ response = self.receive_sdo(bus)
        if response:
            speed = int.from_bytes(response[4:8], byteorder='little', signed=True)
            print(f"{self.motor_name} current speed: {speed} RPM.")
            return speed
        else:
            print(f"Failed to read speed from {self.motor_name}.")
            return None """
    
    def read_mode(self, bus):
        """Read the current operation mode from the motor."""
        self.send_sdo(bus, 0x6061, 0x00, [], 0x40)
        """ response = self.receive_sdo(bus)
        if response:
            mode = response[4]
            print(f"{self.motor_name} current operation mode: {mode}.")
            return mode
        else:
            print(f"Failed to read operation mode from {self.motor_name}.")
            return None """

    def fault_reset(self, bus):
        """Send fault reset command."""
        self.send_sdo(bus, 0x6040, 0x00, [0x80, 0x00], 0x2B)

    def test_method(self, bus, position, speed = 2000):
        speed_bytes = self.int32_to_bytes(speed)
        position_bytes = self.int32_to_bytes(position)
        acc_bytes = self.int32_to_bytes(self.acc*10)
        dec_bytes = self.int32_to_bytes(self.dec*10)

        self.send_sdo(bus, 0x6040, 0x00, [0x06, 0x00], 0x2B)  # Shutdown
        self.send_sdo(bus, 0x6040, 0x00, [0x07, 0x00], 0x2B)  # Switch on
        self.send_sdo(bus, 0x6040, 0x00, [0x0F, 0x00], 0x2B)  # Enable operation
        self.send_sdo(bus, 0x6060, 0x00, [1], 0x2F)
        self.send_sdo(bus, 0x6081, 0x00, speed_bytes, 0x23)  # Set speed for position movement
        self.send_sdo(bus, 0x6083, 0x00, acc_bytes, 0x23)  # Acceleration
        self.send_sdo(bus, 0x6084, 0x00, dec_bytes, 0x23)
        self.send_sdo(bus, 0x607A, 0x00, position_bytes, 0x23)  # Set target position
        self.send_sdo(bus, 0x6040, 0x00, [0x0F, 0x00], 0x2B)  # Start movement
        self.send_sdo(bus, 0x6040, 0x00, [0x1F, 0x00], 0x2B)  
        print(f"{self.motor_name} target position set to {position} counts at speed {speed} RPM.")
    