import can
import time

import can_motor

bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=500000)
#bus.flush_tx_buffer()


motor1 = can_motor.Motor("motor1", 101, "small")
motor2 = can_motor.Motor("motor2", 104, "small")

motor1.fault_reset(bus)
motor2.fault_reset(bus)

""" motor1.set_operation_mode(bus, 1)
motor2.set_operation_mode(bus, 1)

motor1.test_method(bus, position=-10000, speed=2000)
motor2.test_method(bus, position=-10000, speed=2000)

time.sleep(10)

motor1.test_method(bus, position=10000, speed=2000)
motor2.test_method(bus, position=10000, speed=2000)

time.sleep(10)

motor1.set_target_position(bus, -10000)
motor2.set_target_position(bus, -10000) """

motor1.enable_motor(bus)
motor2.enable_motor(bus)

motor1.set_operation_mode(bus, 1)
print(motor1.receive_sdo(bus))
motor2.set_operation_mode(bus, 1)

motor1.set_target_position(bus, position=0, speed=2000)
motor2.set_target_position(bus, position=0, speed=2000)

motor1.read_position(bus)
motor2.read_position(bus)