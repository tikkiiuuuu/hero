import struct
from typing import NamedTuple

class GimbalData(NamedTuple):
    head: bytes
    mode: int
    q: tuple  # (w, x, y, z)
    yaw: float
    yaw_vel: float
    pitch: float
    pitch_vel: float
    bullet_speed: float
    bullet_count: int
    crc16: int

def unpack_gimbal_data(hex_data: str) -> GimbalData:
    """
    解包云台数据
    hex_data: 16进制字符串
    """
    # 将16进制字符串转换为字节
    if isinstance(hex_data, str):
        data_bytes = bytes.fromhex(hex_data)
    else:
        data_bytes = hex_data
    
    # 检查数据长度
    expected_size = 43  # 2+1+16+16+4+2
    if len(data_bytes) != expected_size:
        raise ValueError(f"Data length error: expected {expected_size}, got {len(data_bytes)}")
    
    # 解包数据
    # 格式: 2s B 4f 4f f H H
    unpacked = struct.unpack('<2s B 4f 4f f H H', data_bytes)
    
    return GimbalData(
        head=unpacked[0],
        mode=unpacked[1],
        q=unpacked[2:6],           # 四元数
        yaw=unpacked[6],           # yaw
        yaw_vel=unpacked[7],       # yaw_vel
        pitch=unpacked[8],         # pitch
        pitch_vel=unpacked[9],     # pitch_vel
        bullet_speed=unpacked[10], # bullet_speed
        bullet_count=unpacked[11], # bullet_count
        crc16=unpacked[12]         # crc16
    )

def print_gimbal_data(data: GimbalData):
    """打印解包数据"""
    print("=== Gimbal Data Packet ===")
    print(f"Frame Head: {data.head.decode('ascii')}")
    print(f"Mode: {data.mode}")
    print(f"Quaternion: [{data.q[0]:.6f}, {data.q[1]:.6f}, {data.q[2]:.6f}, {data.q[3]:.6f}]")
    print(f"Yaw: {data.yaw:.2f}, Yaw Velocity: {data.yaw_vel:.2f}")
    print(f"Pitch: {data.pitch:.2f}, Pitch Velocity: {data.pitch_vel:.2f}")
    print(f"Bullet Speed: {data.bullet_speed:.2f}, Count: {data.bullet_count}")
    print(f"CRC16: 0x{data.crc16:04X}")
    print("==========================")

# 使用示例
if __name__ == "__main__":
    hex_data = "535000d1737b3f190d8bbbe5319abcb4223f3e5070c03e0010343ccadf7bbc000000000000704100005454"
    
    try:
        gimbal_data = unpack_gimbal_data(hex_data)
        print_gimbal_data(gimbal_data)
    except Exception as e:
        print(f"Error: {e}")
