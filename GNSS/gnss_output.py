import serial
import pynmea2
import time

def main():
    port = "/dev/ttyAMA0"
    baud = 115200
    
    try:
        ser = serial.Serial(port, baudrate=baud, timeout=1)
        print(f"--- GNSS 实时定位监控启动 ---")
        print(f"--- 端口: {port} | 波特率: {baud} ---")
        print("等待卫星锁定数据...\n")

        while True:
            line = ser.readline().decode('ascii', errors='replace').strip()
 
            # Parse GGA sentences for position, altitude and satellite count.
            if 'GGA' in line:
                try:
                    msg = pynmea2.parse(line)
                    
                    # Accept only fixes with a nonzero quality indicator.
                    if msg.gps_qual > 0:
                        output = (
                            f"时间: {msg.timestamp} | "
                            f"纬度: {msg.latitude:.6f} | "
                            f"经度: {msg.longitude:.6f} | "
                            f"海拔: {msg.altitude}{msg.altitude_units} | "
                            f"卫星: {msg.num_sats}"
                        )
                        print(output)
                    else:
                        print(f"[{time.strftime('%H:%M:%S')}] 正在搜星中... 当前可见卫星: {msg.num_sats}", end='\r')
                        
                except Exception:
                    continue

    except KeyboardInterrupt:
        print("\n\n程序已由用户停止。")
    finally:
        if 'ser' in locals():
            ser.close()

if __name__ == "__main__":
    main()
