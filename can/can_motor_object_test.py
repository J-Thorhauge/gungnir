import can
import time
import asyncio
import can_motor

async def read_messages(bus):
    """Continuously read and process messages from the CAN buffer."""
    while True:
        message = bus.recv(timeout=0)
        while message:
            #Motor 1
            if message.arbitration_id == motor1.rx_id:            
                if message.data[1:3]==b'\x6c\x60': #Speed response
                    motor1.speed = int.from_bytes(message.data[4:8], byteorder='little', signed=True)
                    print(f"[{time.time()}] Motor 1 speed: {motor1.speed}")
                elif message.data[1:3]==b'\x64\x60': #Position response
                    motor1.position = int.from_bytes(message.data[4:8], byteorder='little', signed=True)
                    print(f"[{time.time()}] Motor 1 speed: {motor1.position}")
            #Motor 2
            if message.arbitration_id == motor2.rx_id:
                if message.data[1:3]==b'\x6c\x60': #Speed response
                    motor2.speed = int.from_bytes(message.data[4:8], byteorder='little', signed=True)
                    print(f"[{time.time()}] Motor 2 speed: {motor2.speed}")
                elif message.data[1:3]==b'\x64\x60': #Position response
                    motor2.position = int.from_bytes(message.data[4:8], byteorder='little', signed=True)
                    print(f"[{time.time()}] Motor 2 speed: {motor2.position}")


            message = bus.recv(timeout=0)
        await asyncio.sleep(0.001)

async def send_requests(bus):
    """Periodically send speed requests to both motors."""
    while True:
        motor1.read_position(bus)
        motor2.read_position(bus)
        motor1.read_speed(bus)
        motor2.read_speed(bus)
        print(f"[{time.time()}] Sent speed request!")
        await asyncio.sleep(0.01)

async def main(bus):
    await asyncio.gather(send_requests(bus), read_messages(bus))

# Setup CAN bus
bus = can.interface.Bus(channel='can0', interface='socketcan', bitrate=500000)

# Initialize motors
motor1 = can_motor.Motor("motor1", 101, "small")
motor2 = can_motor.Motor("motor2", 104, "small")

motor1.fault_reset(bus)
motor2.fault_reset(bus)
motor1.enable_motor(bus)
motor2.enable_motor(bus)
motor1.set_operation_mode(bus, 3)
motor2.set_operation_mode(bus, 3)
motor1.set_target_speed(bus, 1000)


# Run both tasks concurrently
asyncio.run(main(bus))