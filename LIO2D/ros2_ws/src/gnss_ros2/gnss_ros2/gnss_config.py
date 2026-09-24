import time
import serial
import os
import re

# Fallback port when hardware discovery provides no GNSS device.
PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 9600
TARGET_BAUD = 115200

# Load the GNSS port selected by hw_handshake.py.
try:
    with open('/tmp/hw_map.env', 'r') as f:
        content = f.read()
        match = re.search(r'GNSS_DEV=([^\n]+)', content)
        if match and match.group(1).strip() != "NONE":
            PORT = match.group(1).strip()
except Exception as e:
    print(f"Failed to read dynamic GNSS port: {e}, using default {PORT}")



def ubx_checksum(data: bytes):
    ck_a = 0
    ck_b = 0
    for b in data:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes([ck_a, ck_b])


def build_ubx(msg_class: int, msg_id: int, payload: bytes) -> bytes:
    body = bytes([msg_class, msg_id]) + len(payload).to_bytes(2, "little") + payload
    return b"\xB5\x62" + body + ubx_checksum(body)


def build_cfg_prt_uart1_115200():
    payload = bytes([
        0x01,             # portID = UART1
        0x00,             # reserved0
        0x00, 0x00,       # txReady
        0xD0, 0x08, 0x00, 0x00,   # mode = 8N1
        0x00, 0xC2, 0x01, 0x00,   # baudRate = 115200
        0x03, 0x00,       # inProtoMask = UBX + NMEA
        0x03, 0x00,       # outProtoMask = UBX + NMEA
        0x00, 0x00,       # flags
        0x00, 0x00        # reserved5
    ])
    return build_ubx(0x06, 0x00, payload)


def build_cfg_rate_10hz():
    payload = bytes([
        0x64, 0x00,   # measRate = 100 ms
        0x01, 0x00,   # navRate = 1
        0x01, 0x00    # timeRef = GPS
    ])
    return build_ubx(0x06, 0x08, payload)


def build_cfg_save():
    payload = bytes([
        0x00, 0x00, 0x00, 0x00,   # clearMask
        0xFF, 0xFF, 0x00, 0x00,   # saveMask
        0x17, 0x00, 0x00, 0x00    # devMask: BBR + Flash + EEPROM
    ])
    return build_ubx(0x06, 0x09, payload)


def configure_gnss():
    msg_prt = build_cfg_prt_uart1_115200()
    msg_rate = build_cfg_rate_10hz()
    msg_save = build_cfg_save()

    ser = serial.Serial(PORT, DEFAULT_BAUD, timeout=0.2)
    time.sleep(1.0)

    ser.write(msg_prt)
    ser.flush()
    time.sleep(0.5)

    ser.close()
    time.sleep(0.5)

    ser = serial.Serial(PORT, TARGET_BAUD, timeout=0.2)
    time.sleep(0.8)

    ser.write(msg_rate)
    ser.flush()
    time.sleep(0.5)

    ser.write(msg_save)
    ser.flush()
    time.sleep(0.5)

    ser.close()

