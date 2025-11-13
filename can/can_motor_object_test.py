import can
import time

import can_motor

bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=500000)

motor1 = can_motor.Motor("motor1", 101, "small")
motor2 = can_motor.Motor("motor2", 104, "small")

motor1.fault_reset(bus)
motor2.fault_reset(bus)

motor1.enable_motor(bus)
motor2.enable_motor(bus)

motor1.set_operation_mode(bus, 3)
motor2.set_operation_mode(bus, 3)

while True:
    motor1.set_target_speed(bus, 4000)
    motor2.set_target_speed(bus, 0)
    time.sleep(2)

    motor1.set_target_speed(bus, 0)
    motor2.set_target_speed(bus, 4000)
    time.sleep(2)