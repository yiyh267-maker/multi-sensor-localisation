import serial
import time
import os

def find_devices():
    all_devs = [f"/dev/{d}" for d in os.listdir('/dev') if d.startswith('ttyUSB')]
    mapping = {"LIDAR": "NONE", "IMU": "NONE", "GNSS": "NONE"}

    for dev in all_devs:
        print(f"🕵️ 正在强行握手: {dev}")
        try:
            # Probe the RPLIDAR response header before classifying streaming devices.
            with serial.Serial(dev, 115200, timeout=0.2) as ser:
                ser.write(b'\xa5\x50')
                res = ser.read(10)
                if len(res) >= 2 and res[0] == 0xa5 and res[1] == 0x5a:
                    mapping["LIDAR"] = dev
                    print(f"✅ [LIDAR] 锁定: {dev}")
                    continue

            with serial.Serial(dev, 115200, timeout=0.5) as ser:
                sample = ser.read(300)
                
                # Treat a short stream with an AT/OK response as a GNSS port candidate.
                if len(sample) < 10:
                    ser.write(b'AT\r\n')
                    time.sleep(0.1)
                    if b'OK' in ser.read(50):
                        mapping["GNSS"] = dev
                        print(f"✅ [GNSS] 锁定 (AT控制口): {dev}")
                    continue

                # Recognize GNSS candidates by NMEA talker prefixes.
                if b'$G' in sample or b'$B' in sample:
                    if mapping["GNSS"] == "NONE":
                        mapping["GNSS"] = dev
                        print(f"✅ [GNSS] 锁定 (NMEA数据流): {dev}")
                    continue
                
                # Use VN bytes or a long non-NMEA stream as an IMU heuristic.
                if b'\x56\x4e' in sample or b'VN' in sample:
                    mapping["IMU"] = dev
                    print(f"✅ [IMU] 锁定 (VectorNav特征): {dev}")
                    continue
                elif len(sample) > 50:
                    mapping["IMU"] = dev
                    print(f"✅ [IMU] 锁定 (高频二进制流): {dev}")
                    continue

        except Exception as e:
            # Skip ports that raise an exception during probing.
            continue

    return mapping

if __name__ == "__main__":
    results = find_devices()
    with open("/tmp/hw_map.env", "w") as f:
        f.write(f"export LIDAR_DEV={results['LIDAR']}\n")
        f.write(f"export IMU_DEV={results['IMU']}\n")
        f.write(f"export GNSS_DEV={results['GNSS']}\n")
    
    print("-" * 30)
    print("最终物理端口映射表:")
    print(f"LIDAR -> {results['LIDAR']}")
    print(f"IMU   -> {results['IMU']}")
    print(f"GNSS  -> {results['GNSS']}")
    print("-" * 30)
