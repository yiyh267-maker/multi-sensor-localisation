import smbus
import struct
import time
import math
import sys

import evdev
from evdev import InputDevice, ecodes

I2C_BUS_ID = 1
MOTOR_MODEL_ADDR = 0x26
MOTOR_TYPE = 1

JOY_DEADZONE = 6000

# Nominal signed 16-bit joystick range.
JOY_MAX = 32767.0

MAX_LINEAR_CMD = 500

MAX_ROTATE_CMD = 500

# Motor command resend interval in seconds.
RESEND_DT = 0.05

MOTOR_TYPE_REG = 0x01
MOTOR_DEADZONE_REG = 0x02
MOTOR_PLUSELINE_REG = 0x03
MOTOR_PLUSEPHASE_REG = 0x04
WHEEL_DIA_REG = 0x05
SPEED_CONTROL_REG = 0x06
PWM_CONTROL_REG = 0x07

bus = smbus.SMBus(I2C_BUS_ID)

def i2c_write(addr, reg, data):
    bus.write_i2c_block_data(addr, reg, data)

def float_to_bytes(value):
    return list(struct.pack('<f', value))

def int16_to_bytes(value):
    value = int(round(value))
    value &= 0xFFFF
    return [(value >> 8) & 0xFF, value & 0xFF]

def set_motor_type(data):
    i2c_write(MOTOR_MODEL_ADDR, MOTOR_TYPE_REG, [data])

def set_motor_deadzone(data):
    i2c_write(MOTOR_MODEL_ADDR, MOTOR_DEADZONE_REG, int16_to_bytes(data))

def set_pluse_line(data):
    i2c_write(MOTOR_MODEL_ADDR, MOTOR_PLUSELINE_REG, int16_to_bytes(data))

def set_pluse_phase(data):
    i2c_write(MOTOR_MODEL_ADDR, MOTOR_PLUSEPHASE_REG, int16_to_bytes(data))

def set_wheel_dis(data):
    i2c_write(MOTOR_MODEL_ADDR, WHEEL_DIA_REG, float_to_bytes(data))

def set_motor_parameter():
    if MOTOR_TYPE == 1:
        set_motor_type(1)
        time.sleep(0.1)
        set_pluse_phase(30)
        time.sleep(0.1)
        set_pluse_line(11)
        time.sleep(0.1)
        set_wheel_dis(67.00)
        time.sleep(0.1)
        set_motor_deadzone(1600)
        time.sleep(0.1)

    elif MOTOR_TYPE == 2:
        set_motor_type(2)
        time.sleep(0.1)
        set_pluse_phase(20)
        time.sleep(0.1)
        set_pluse_line(13)
        time.sleep(0.1)
        set_wheel_dis(48.00)
        time.sleep(0.1)
        set_motor_deadzone(1200)
        time.sleep(0.1)

    elif MOTOR_TYPE == 3:
        set_motor_type(3)
        time.sleep(0.1)
        set_pluse_phase(45)
        time.sleep(0.1)
        set_pluse_line(13)
        time.sleep(0.1)
        set_wheel_dis(68.00)
        time.sleep(0.1)
        set_motor_deadzone(1250)
        time.sleep(0.1)

    elif MOTOR_TYPE == 4:
        set_motor_type(4)
        time.sleep(0.1)
        set_pluse_phase(48)
        time.sleep(0.1)
        set_motor_deadzone(1000)
        time.sleep(0.1)

    elif MOTOR_TYPE == 5:
        set_motor_type(1)
        time.sleep(0.1)
        set_pluse_phase(40)
        time.sleep(0.1)
        set_pluse_line(11)
        time.sleep(0.1)
        set_wheel_dis(67.00)
        time.sleep(0.1)
        set_motor_deadzone(1600)
        time.sleep(0.1)

    else:
        raise ValueError(f"Unsupported MOTOR_TYPE: {MOTOR_TYPE}")

def control_speed_raw(m1, m2, m3, m4):
    payload = (
        int16_to_bytes(m1) +
        int16_to_bytes(m2) +
        int16_to_bytes(m3) +
        int16_to_bytes(m4)
    )
    i2c_write(MOTOR_MODEL_ADDR, SPEED_CONTROL_REG, payload)

def control_pwm_raw(m1, m2, m3, m4):
    payload = (
        int16_to_bytes(m1) +
        int16_to_bytes(m2) +
        int16_to_bytes(m3) +
        int16_to_bytes(m4)
    )
    i2c_write(MOTOR_MODEL_ADDR, PWM_CONTROL_REG, payload)

def stop_all():
    if MOTOR_TYPE == 4:
        control_pwm_raw(0, 0, 0, 0)
    else:
        control_speed_raw(0, 0, 0, 0)

# Forward-positive wheels map to M1=-RR, M2=LR, M3=RF, M4=-LF.
def wheel_to_raw(lf, lr, rf, rr):
    m1 = -int(round(rr))
    m2 =  int(round(lr))
    m3 =  int(round(rf))
    m4 = -int(round(lf))
    return m1, m2, m3, m4

def send_wheel_speed(lf, lr, rf, rr):
    m1, m2, m3, m4 = wheel_to_raw(lf, lr, rf, rr)
    if MOTOR_TYPE == 4:
        control_pwm_raw(m1, m2, m3, m4)
    else:
        control_speed_raw(m1, m2, m3, m4)

def set_diff_drive(left_cmd, right_cmd):
    lf = left_cmd
    lr = left_cmd
    rf = right_cmd
    rr = right_cmd
    send_wheel_speed(lf, lr, rf, rr)

def apply_deadzone(value, deadzone=JOY_DEADZONE):
    if abs(value) < deadzone:
        return 0
    return value

def normalize_axis(value):
    value = apply_deadzone(value)
    value = max(-32768, min(32767, value))
    return value / JOY_MAX

def clamp(value, low, high):
    return max(low, min(high, value))

def sticks_to_command(left_y, right_x):
    # Invert joystick axes before mixing translation and rotation.
    linear = -normalize_axis(left_y)
    rotate = -normalize_axis(right_x)

    linear_cmd = linear * MAX_LINEAR_CMD
    rotate_cmd = rotate * MAX_ROTATE_CMD

    # Mix left-stick Y and right-stick X into differential wheel commands.
    left_cmd = linear_cmd - rotate_cmd
    right_cmd = linear_cmd + rotate_cmd

    left_cmd = clamp(left_cmd, -1000, 1000)
    right_cmd = clamp(right_cmd, -1000, 1000)

    return int(left_cmd), int(right_cmd), linear, rotate

def main():
    print("Configuring motor parameters...")
    set_motor_parameter()
    time.sleep(0.2)
    stop_all()

    gamepad_path = None
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    for d in devices:
        # Select the first input device with absolute-axis events.
        if ecodes.EV_ABS in d.capabilities():
            gamepad_path = d.path
            break
            
    if gamepad_path is None:
        print("❌ 未检测到手柄！请确认手柄接收器已插紧，或蓝牙已连接。")
        sys.exit(1)

    print(f"Opening gamepad (Auto-detected): {gamepad_path}")
    dev = InputDevice(gamepad_path)
    print(f"Connected to: {dev.name}")
    print("Control mapping:")
    print("  Left stick Y  -> forward / backward")
    print("  Right stick X -> rotate in place")
    print("Press Ctrl+C to quit.")

    left_y = 0
    right_x = 0

    last_send_time = 0.0
    current_left_cmd = 0
    current_right_cmd = 0

    try:
        while True:
            event = dev.read_one()

            if event is not None:
                if event.type == ecodes.EV_ABS:
                    if event.code == ecodes.ABS_Y:
                        left_y = event.value
                    elif event.code == ecodes.ABS_RX:
                        right_x = event.value

            now = time.time()
            if now - last_send_time >= RESEND_DT:
                left_cmd, right_cmd, linear_norm, rotate_norm = sticks_to_command(left_y, right_x)

                # Suppress small residual wheel commands.
                if abs(left_cmd) < 20 and abs(right_cmd) < 20:
                    left_cmd = 0
                    right_cmd = 0

                current_left_cmd = left_cmd
                current_right_cmd = right_cmd

                set_diff_drive(current_left_cmd, current_right_cmd)

                print(
                    f"\rABS_Y={left_y:6d}  ABS_RX={right_x:6d}  "
                    f"L={current_left_cmd:5d}  R={current_right_cmd:5d}  "
                    f"linear={linear_norm:+.2f}  rotate={rotate_norm:+.2f}",
                    end="",
                    flush=True
                )

                last_send_time = now

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\nStopping motors...")
        stop_all()
        time.sleep(0.1)
        print("Exited.")

if __name__ == "__main__":
    main()
