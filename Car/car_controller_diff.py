import smbus
import struct
import time
import math
import sys

I2C_BUS_ID = 1
MOTOR_MODEL_ADDR = 0x26

# 1: accumulated encoder counts; 2: encoder delta over 10 ms.
UPLOAD_DATA = 1

# Motor types: 1=520, 2=310, 3=encoded TT, 4=DC TT, 5=L-type 520.
MOTOR_TYPE = 1

WHEEL_DIAMETER_M = 0.067
TRACK_WIDTH_M = 0.118
WHEEL_BASE_M = 0.100
COUNTS_PER_REV = 330.0

DEFAULT_LINEAR_CMD = 220
DEFAULT_ROTATE_CMD = 600
CONTROL_DT = 0.05
STOP_SETTLE_TIME = 0.20

ROTATE_MIN_RUNTIME = 0.30
ROTATE_MAX_RUNTIME = 8.00
ROTATE_FINISH_TOL_DEG = 3.0

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

pose_x = 0.0
pose_y = 0.0
pose_yaw = 0.0

last_raw_counts = None
last_odom_time = None

def i2c_write(addr, reg, data):
    bus.write_i2c_block_data(addr, reg, data)


def i2c_read(addr, reg, length):
    return bus.read_i2c_block_data(addr, reg, length)


def float_to_bytes(value):
    return list(struct.pack('<f', value))


def int16_to_bytes(value):
    value = int(value)
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
    m1 = -int(rr)
    m2 =  int(lr)
    m3 =  int(rf)
    m4 = -int(lf)
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


def move_forward(cmd=DEFAULT_LINEAR_CMD):
    set_diff_drive(cmd, cmd)


def move_backward(cmd=DEFAULT_LINEAR_CMD):
    set_diff_drive(-cmd, -cmd)


def rotate_ccw(cmd=DEFAULT_ROTATE_CMD):
    set_diff_drive(-cmd, cmd)


def rotate_cw(cmd=DEFAULT_ROTATE_CMD):
    set_diff_drive(cmd, -cmd)


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


def format_raw_counts(counts):
    return f"M1={counts[0]}, M2={counts[1]}, M3={counts[2]}, M4={counts[3]}"


def raw_to_wheel_counts(raw_counts):
    m1, m2, m3, m4 = raw_counts
    lf = -m4
    lr =  m2
    rf =  m3
    rr = -m1
    return [lf, lr, rf, rr]


def format_wheel_counts(wheel_counts):
    lf, lr, rf, rr = wheel_counts
    return f"LF={lf}, LR={lr}, RF={rf}, RR={rr}"


def normalize_angle(rad):
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


def counts_to_distance(count_delta):
    rev = count_delta / COUNTS_PER_REV
    return rev * (math.pi * WHEEL_DIAMETER_M)


def reset_odometry():
    global pose_x, pose_y, pose_yaw, last_raw_counts, last_odom_time
    pose_x = 0.0
    pose_y = 0.0
    pose_yaw = 0.0
    last_raw_counts = read_all_encoder_list()
    last_odom_time = time.time()


def update_odometry(verbose=False):
    global pose_x, pose_y, pose_yaw, last_raw_counts, last_odom_time

    now = time.time()
    raw_counts = read_all_encoder_list()

    if last_raw_counts is None:
        last_raw_counts = raw_counts
        last_odom_time = now
        return {
            "x": pose_x,
            "y": pose_y,
            "yaw": pose_yaw,
            "v": 0.0,
            "w": 0.0,
            "ds": 0.0,
            "dtheta": 0.0,
            "raw": raw_counts,
            "wheel": raw_to_wheel_counts(raw_counts),
        }

    dt = now - last_odom_time
    if dt <= 0:
        dt = CONTROL_DT

    raw_delta = [raw_counts[i] - last_raw_counts[i] for i in range(4)]
    wheel_delta_counts = raw_to_wheel_counts(raw_delta)

    lf_d = counts_to_distance(wheel_delta_counts[0])
    lr_d = counts_to_distance(wheel_delta_counts[1])
    rf_d = counts_to_distance(wheel_delta_counts[2])
    rr_d = counts_to_distance(wheel_delta_counts[3])

    left_d = 0.5 * (lf_d + lr_d)
    right_d = 0.5 * (rf_d + rr_d)

    ds = 0.5 * (left_d + right_d)
    dtheta = (right_d - left_d) / TRACK_WIDTH_M

    # Integrate wheel displacement using the midpoint heading.
    mid_yaw = pose_yaw + 0.5 * dtheta
    pose_x += ds * math.cos(mid_yaw)
    pose_y += ds * math.sin(mid_yaw)
    pose_yaw = normalize_angle(pose_yaw + dtheta)

    v = ds / dt
    w = dtheta / dt

    last_raw_counts = raw_counts
    last_odom_time = now

    if verbose:
        print("RAW   :", format_raw_counts(raw_counts))
        print("WHEEL :", format_wheel_counts(raw_to_wheel_counts(raw_counts)))
        print(
            f"ODOM  : x={pose_x:.4f} m, y={pose_y:.4f} m, "
            f"yaw={math.degrees(pose_yaw):.2f} deg, "
            f"v={v:.4f} m/s, w={math.degrees(w):.2f} deg/s"
        )
        print("-" * 70)

    return {
        "x": pose_x,
        "y": pose_y,
        "yaw": pose_yaw,
        "v": v,
        "w": w,
        "ds": ds,
        "dtheta": dtheta,
        "raw": raw_counts,
        "wheel": raw_to_wheel_counts(raw_counts),
    }


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


def hold_still(duration_sec=1.0, verbose=True):
    stop_all()
    odom = None
    t_end = time.time() + duration_sec
    while time.time() < t_end:
        odom = update_odometry(verbose=verbose)
        time.sleep(CONTROL_DT)
    return odom


def drive_straight_for(duration_sec, forward=True, cmd=DEFAULT_LINEAR_CMD, verbose=True):
    if forward:
        move_forward(cmd)
    else:
        move_backward(cmd)

    t_end = time.time() + duration_sec
    while time.time() < t_end:
        update_odometry(verbose=verbose)
        time.sleep(CONTROL_DT)

    stop_all()
    time.sleep(STOP_SETTLE_TIME)
    return update_odometry(verbose=verbose)


def rotate_in_place_by_angle(angle_deg, cmd=DEFAULT_ROTATE_CMD, verbose=True):
    if angle_deg == 0:
        return update_odometry(verbose=verbose)

    target_rad = math.radians(angle_deg)
    accumulated_rad = 0.0
    start_time = time.time()

    if angle_deg > 0:
        rotate_ccw(cmd)
    else:
        rotate_cw(cmd)

    while True:
        odom = update_odometry(verbose=verbose)
        accumulated_rad += odom["dtheta"]

        elapsed = time.time() - start_time
        accumulated_deg = math.degrees(accumulated_rad)
        target_abs_deg = abs(angle_deg)
        accum_abs_deg = abs(accumulated_deg)

        print(
            f"ROT   : target={angle_deg:.2f} deg, "
            f"accum={accumulated_deg:.2f} deg, "
            f"remain={target_abs_deg - accum_abs_deg:.2f} deg"
        )
        print("=" * 70)

        if elapsed >= ROTATE_MIN_RUNTIME:
            if accum_abs_deg >= (target_abs_deg - ROTATE_FINISH_TOL_DEG):
                break

        if elapsed > ROTATE_MAX_RUNTIME:
            print("Rotation timeout reached.")
            break

        time.sleep(CONTROL_DT)

    stop_all()
    time.sleep(STOP_SETTLE_TIME)
    return update_odometry(verbose=verbose)


def print_usage():
    print("Usage:")
    print("  sudo python3 car_controller_diff.py init")
    print("  sudo python3 car_controller_diff.py odom")
    print("  sudo python3 car_controller_diff.py stop")
    print("  sudo python3 car_controller_diff.py forward <seconds> [cmd]")
    print("  sudo python3 car_controller_diff.py backward <seconds> [cmd]")
    print("  sudo python3 car_controller_diff.py rotate <angle_deg> [cmd]")
    print("")
    print("Examples:")
    print("  sudo python3 car_controller_diff.py init")
    print("  sudo python3 car_controller_diff.py forward 2.0 220")
    print("  sudo python3 car_controller_diff.py backward 1.5 220")
    print("  sudo python3 car_controller_diff.py rotate 90 800")
    print("  sudo python3 car_controller_diff.py rotate -90 800")
    print("  sudo python3 car_controller_diff.py odom")


def main():
    if len(sys.argv) < 2:
        print_usage()
        sys.exit(1)

    cmd = sys.argv[1].lower()

    try:
        if cmd == "init":
            print("Configuring motor parameters...")
            set_motor_parameter()
            time.sleep(0.2)
            reset_odometry()
            print("Done.")
            print(f"I2C addr      : 0x{MOTOR_MODEL_ADDR:02X}")
            print(f"wheel diameter: {WHEEL_DIAMETER_M:.3f} m")
            print(f"track width   : {TRACK_WIDTH_M:.3f} m")
            print(f"wheel base    : {WHEEL_BASE_M:.3f} m")
            print(f"counts/rev    : {COUNTS_PER_REV}")
            print("Odometry reset complete.")

        elif cmd == "stop":
            stop_all()
            print("All motors stopped.")

        elif cmd == "odom":
            reset_odometry()
            print("Reading odometry. Press Ctrl+C to stop.")
            while True:
                update_odometry(verbose=True)
                time.sleep(CONTROL_DT)

        elif cmd == "forward":
            if len(sys.argv) < 3:
                print("Missing duration.")
                sys.exit(1)
            duration_sec = float(sys.argv[2])
            speed_cmd = int(sys.argv[3]) if len(sys.argv) >= 4 else DEFAULT_LINEAR_CMD

            set_motor_parameter()
            reset_odometry()
            print(f"Forward for {duration_sec:.2f}s, cmd={speed_cmd}")
            final_odom = drive_straight_for(duration_sec, forward=True, cmd=speed_cmd, verbose=True)
            print("Final:", final_odom)

        elif cmd == "backward":
            if len(sys.argv) < 3:
                print("Missing duration.")
                sys.exit(1)
            duration_sec = float(sys.argv[2])
            speed_cmd = int(sys.argv[3]) if len(sys.argv) >= 4 else DEFAULT_LINEAR_CMD

            set_motor_parameter()
            reset_odometry()
            print(f"Backward for {duration_sec:.2f}s, cmd={speed_cmd}")
            final_odom = drive_straight_for(duration_sec, forward=False, cmd=speed_cmd, verbose=True)
            print("Final:", final_odom)

        elif cmd == "rotate":
            if len(sys.argv) < 3:
                print("Missing angle.")
                sys.exit(1)
            angle_deg = float(sys.argv[2])
            speed_cmd = int(sys.argv[3]) if len(sys.argv) >= 4 else DEFAULT_ROTATE_CMD

            set_motor_parameter()
            reset_odometry()
            hold_still(0.5, verbose=True)
            print(f"Rotate in place by {angle_deg:.2f} deg, cmd={speed_cmd}")
            final_odom = rotate_in_place_by_angle(angle_deg, cmd=speed_cmd, verbose=True)
            print("Final:", final_odom)

        else:
            print_usage()
            sys.exit(1)

    except KeyboardInterrupt:
        print("\nInterrupted. Stopping motors...")
        stop_all()
        sys.exit(0)


if __name__ == "__main__":
    main()
