#!/usr/bin/env python3
import serial
import struct
import sys

# 串口配置（根据实际情况修改）
PORT = '/dev/gimbal'  # 或 '/dev/ttyACM0' 等
BAUDRATE = 115200

def main():
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        print(f"已连接串口: {PORT}, 波特率: {BAUDRATE}")
        print("等待数据...\n")
    except Exception as e:
        print(f"打开串口失败: {e}")
        sys.exit(1)

    error_count = 0
    while True:
        try:
            # 读取head (2字节)
            head = ser.read(2)
            if len(head) != 2:
                error_count += 1
                continue
            
            if head[0] != ord('S') or head[1] != ord('P'):
                # head不匹配，尝试重新对齐
                ser.read(1)  # 跳过1字节
                error_count += 1
                continue
            
            # 读取剩余数据 (4字节float + 2字节uint16)
            data = ser.read(6)
            if len(data) != 6:
                error_count += 1
                continue
            
            # 解析数据: < 表示小端序
            timestamp, crc16 = struct.unpack('<fH', data)

            raw_data = head + data
            print(f"原始数据: {raw_data.hex()}")
            
            # 打印数据
            print(f"timestamp: {timestamp:.2f} ms, CRC16: 0x{crc16:04X}, errors: {error_count}")
            error_count = 0
            
        except KeyboardInterrupt:
            print("\n退出...")
            break
        except Exception as e:
            error_count += 1
            print(f"错误: {e}, error_count: {error_count}")
    
    ser.close()

if __name__ == '__main__':
    main()