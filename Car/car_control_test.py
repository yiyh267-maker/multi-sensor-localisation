import smbus
import struct
import time
import sys

I2C_BUS_ID = 1
MOTOR_MODEL_ADDR = 0x26

# 1: accumulated encoder counts; 2: encoder delta over 10 ms.
UPLOAD_DATA = 2

# Motor types: 1=520, 2=310, 3=encoded TT, 4=DC TT, 5=L-type 520.
MOTOR_TYPE = 1

TEST_SPEED = 220

ACTION_TIME = 2.0

MONITOR_DT = 0.10


MOTOR_TYPE_REG = 0x01
MOTOR_DEADZONE_REG = 0x02
MOTOR_PLUSELINE_REG = 0x03
MOTOR_PLUSEPHASE_REG = 0x04
WHEEL_DIA_REG = 0x05
SPEED_CONTROL_REG = 0x06
PWM_CONTROL_REG = 0x07

READ_TEN_M1_ENCODER_REG = 0x10
READ_TEN_M2_ENCODER_REG = 0x11
READ_TEN_M3_ENCODER_REG = 0x12
READ_TEN_M4_ENCODER_REG = 0x13

READ_ALLHIGH_M1_REG = 0x20
READ_ALLLOW_M1_REG = 0x21
READ_ALLHIGH_M2_REG = 0x22
READ_ALLLOW_M2_REG = 0x23
READ_ALLHIGH_M3_REG = 0x24
READ_ALLLOW_M3_REG = 0x25
READ_ALLHIGH_M4_REG = 0x26
READ_ALLLOW_M4_REG = 0x27


bus = smbus.SMBus(I2C_BUS_ID)
encoder_offset = [0, 0, 0, 0]
encoder_now = [0, 0, 0, 0]


def i2c_write(addr, reg, data):
    bus.write_i2c_block_data(addr, reg, data)


def i2c_read(addr, reg, length):
    return bus.read_i2c_block_data(addr, reg, length)


def float_to_bytes(value):
    return struct.pack('<f', value)


def int16_to_bytes(value):
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
    i2c_write(MOTOR_MODEL_ADDR, WHEEL_DIA_REG, list(float_to_bytes(data)))


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
def wheel_to_motor(lf, lr, rf, rr):
    m1 = -rr
    m2 =  lr
    m3 =  rf
    m4 = -lf
    return m1, m2, m3, m4


def control_wheel_speed(lf, lr, rf, rr):
    m1, m2, m3, m4 = wheel_to_motor(lf, lr, rf, rr)
    control_speed_raw(m1, m2, m3, m4)


def read_10_encoder_list():
    global encoder_offset
    out = []

    for i in range(4):
        reg = READ_TEN_M1_ENCODER_REG + i
        buf = i2c_read(MOTOR_MODEL_ADDR, reg, 2)

        value = (buf[0] << 8) | buf[1]
        if value & 0x8000:
            value -= 0x10000

        encoder_offset[i] = value
        out.append(value)

    return out


def read_all_encoder_list():
    global encoder_now
    out = []

    for i in range(4):
        high_reg = READ_ALLHIGH_M1_REG + (i * 2)
        low_reg = READ_ALLLOW_M1_REG + (i * 2)

        high_buf = i2c_read(MOTOR_MODEL_ADDR, high_reg, 2)
        low_buf = i2c_read(MOTOR_MODEL_ADDR, low_reg, 2)

        high_val = (high_buf[0] << 8) | high_buf[1]
        low_val = (low_buf[0] << 8) | low_buf[1]

        value = (high_val << 16) | low_val
        if value >= 0x80000000:
            value -= 0x100000000

        encoder_now[i] = value
        out.append(value)

    return out


def read_encoder_list():
    if UPLOAD_DATA == 1:
        return read_all_encoder_list()
    return read_10_encoder_list()


def raw_to_wheel(raw_m):
    m1, m2, m3, m4 = raw_m

    lf = -m4
    lr =  m2
    rf =  m3
    rr = -m1

    return [lf, lr, rf, rr]


def fmt_raw(raw_m):
    return f"M1={raw_m[0]}, M2={raw_m[1]}, M3={raw_m[2]}, M4={raw_m[3]}"


def fmt_wheel(w):
    return f"LF={w[0]}, LR={w[1]}, RF={w[2]}, RR={w[3]}"


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


def run_action(name, lf, lr, rf, rr, seconds):
    print(f"\n=== ACTION: {name} ===")
    print(f"CMD WHEEL : LF={lf}, LR={lr}, RF={rf}, RR={rr}")

    start = time.time()
    while time.time() - start < seconds:
        control_wheel_speed(lf, lr, rf, rr)
        raw_m = read_encoder_list()
        wheel = raw_to_wheel(raw_m)

        print(f"RAW   : {fmt_raw(raw_m)}")
        print(f"WHEEL : {fmt_wheel(wheel)}")
        print("-" * 56)

        time.sleep(MONITOR_DT)

    stop_all()
    time.sleep(0.3)


def forward(speed=TEST_SPEED, seconds=ACTION_TIME):
    run_action("FORWARD", speed, speed, speed, speed, seconds)


def backward(speed=TEST_SPEED, seconds=ACTION_TIME):
    run_action("BACKWARD", -speed, -speed, -speed, -speed, seconds)


def left_shift(speed=TEST_SPEED, seconds=ACTION_TIME):
    # Lateral wheel pattern for mecanum or omni wheels.
    run_action("LEFT_SHIFT", -speed, speed, speed, -speed, seconds)


def right_shift(speed=TEST_SPEED, seconds=ACTION_TIME):
    # Lateral wheel pattern for mecanum or omni wheels.
    run_action("RIGHT_SHIFT", speed, -speed, -speed, speed, seconds)


def rotate_ccw(speed=TEST_SPEED, seconds=ACTION_TIME):
    run_action("ROTATE_CCW", -speed, -speed, speed, speed, seconds)


def rotate_cw(speed=TEST_SPEED, seconds=ACTION_TIME):
    run_action("ROTATE_CW", speed, speed, -speed, -speed, seconds)


def print_usage():
    print("Usage:")
    print("  sudo python3 car_control_test.py init")
    print("  sudo python3 car_control_test.py fwd")
    print("  sudo python3 car_control_test.py back")
    print("  sudo python3 car_control_test.py left")
    print("  sudo python3 car_control_test.py right")
    print("  sudo python3 car_control_test.py ccw")
    print("  sudo python3 car_control_test.py cw")
    print("  sudo python3 car_control_test.py stop")


def main():
    if len(sys.argv) < 2:
        print_usage()
        return

    cmd = sys.argv[1].lower()

    if cmd == "init":
        print("configuring motor parameters...")
        set_motor_parameter()
        print("done")
        return

    print("configuring motor parameters...")
    set_motor_parameter()
    print("done")

    if cmd == "fwd":
        forward()
    elif cmd == "back":
        backward()
    elif cmd == "left":
        left_shift()
    elif cmd == "right":
        right_shift()
    elif cmd == "ccw":
        rotate_ccw()
    elif cmd == "cw":
        rotate_cw()
    elif cmd == "stop":
        stop_all()
    else:
        print_usage()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nstopping motors...")
        stop_all()
        time.sleep(0.1)
        sys.exit(0)
