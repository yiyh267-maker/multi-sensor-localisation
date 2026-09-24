#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import signal
import sys
import smbus


I2C_BUS_ID = 1
MOTOR_MODEL_ADDR = 0x26
READ_ALLHIGH_M1_REG = 0x30       # High-word register for accumulated encoder counts.
READ_ALLLOW_M1_REG  = 0x31       # Low-word register for accumulated encoder counts.

READ_INTERVAL = 0.10             # Polling interval in seconds.


bus = smbus.SMBus(I2C_BUS_ID)


running = True

zero_m1 = 0
zero_m2 = 0
zero_m3 = 0
zero_m4 = 0



def handle_sigint(sig, frame):
    global running
    running = False
    print("\n收到 Ctrl+C，程序退出中...")
    try:
        bus.close()
    except Exception:
        pass
    sys.exit(0)


signal.signal(signal.SIGINT, handle_sigint)


def i2c_read(addr: int, reg: int, length: int):
    """Read a block of bytes from an I2C register."""
    return bus.read_i2c_block_data(addr, reg, length)



def read_all_encoder_raw():
    """Return signed accumulated counts for M1 through M4."""
    values = []

    for i in range(4):
        high_reg = READ_ALLHIGH_M1_REG + (i * 2)
        low_reg  = READ_ALLLOW_M1_REG  + (i * 2)

        high_buf = i2c_read(MOTOR_MODEL_ADDR, high_reg, 2)
        low_buf  = i2c_read(MOTOR_MODEL_ADDR, low_reg, 2)

        high_val = (high_buf[0] << 8) | high_buf[1]
        low_val  = (low_buf[0]  << 8) | low_buf[1]

        encoder_val = (high_val << 16) | low_val

        # Decode a signed 32-bit encoder count.
        if encoder_val >= 0x80000000:
            encoder_val -= 0x100000000

        values.append(encoder_val)

    return tuple(values)



def capture_zero():
    """Store the current encoder counts as the zero reference."""
    global zero_m1, zero_m2, zero_m3, zero_m4

    m1, m2, m3, m4 = read_all_encoder_raw()

    zero_m1 = m1
    zero_m2 = m2
    zero_m3 = m3
    zero_m4 = m4

    print("已记录零点:")
    print(f"  zero_m1 = {zero_m1}")
    print(f"  zero_m2 = {zero_m2}")
    print(f"  zero_m3 = {zero_m3}")
    print(f"  zero_m4 = {zero_m4}")



def read_encoder_adjusted():
    """Return M1 through M4 counts relative to the zero reference."""
    m1, m2, m3, m4 = read_all_encoder_raw()

    adj_m1 = m1 - zero_m1
    adj_m2 = m2 - zero_m2
    adj_m3 = m3 - zero_m3
    adj_m4 = m4 - zero_m4

    return adj_m1, adj_m2, adj_m3, adj_m4



def map_to_wheels(adj_m1, adj_m2, adj_m3, adj_m4):
    """Return forward-positive LF, LR, RF and RR counts."""
    lf = -adj_m4
    lr =  adj_m2
    rf =  adj_m3
    rr = -adj_m1

    return lf, lr, rf, rr



def main():
    print("正在连接 I2C 电机驱动板...")
    time.sleep(0.2)

    raw_m1, raw_m2, raw_m3, raw_m4 = read_all_encoder_raw()
    print("首次读取成功:")
    print(f"  M1={raw_m1}, M2={raw_m2}, M3={raw_m3}, M4={raw_m4}")

    capture_zero()

    print("\n开始循环读取编码器数据，按 Ctrl+C 退出。\n")

    while running:
        raw_m1, raw_m2, raw_m3, raw_m4 = read_all_encoder_raw()

        adj_m1, adj_m2, adj_m3, adj_m4 = read_encoder_adjusted()

        lf, lr, rf, rr = map_to_wheels(adj_m1, adj_m2, adj_m3, adj_m4)

        print("--------------------------------------------------")
        print(f"RAW   : M1={raw_m1}, M2={raw_m2}, M3={raw_m3}, M4={raw_m4}")
        print(f"ADJ   : M1={adj_m1}, M2={adj_m2}, M3={adj_m3}, M4={adj_m4}")
        print(f"WHEEL : LF={lf}, LR={lr}, RF={rf}, RR={rr}")

        time.sleep(READ_INTERVAL)


if __name__ == "__main__":
    main()
