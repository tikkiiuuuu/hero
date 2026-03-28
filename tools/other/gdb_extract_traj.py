#!/usr/bin/env python3
"""
GDB Python 脚本：自动化提取 plan_with_ruckig 中的轨迹数据
使用方法：
  (gdb) source tools/other/gdb_extract_traj.py
  (gdb) traj_tool start   # 开启自动提取(在断点处不停车，自动保存)
  (gdb) traj_tool stop    # 关闭自动提取
  (gdb) traj_tool once    # 手动提取一次当前帧
"""

import gdb
import json
import os
import struct
from datetime import datetime

# 全局上下文管理
class ExtractContext:
    def __init__(self):
        self.bp = None
        self.counter = 0
        self.output_dir = "/tmp/gdb_traj_data"
        self.horizon = 100  # 预测时域长度
        self.rows = 4       # 状态维度 (yaw, v_yaw, pitch, v_pitch)
        
        os.makedirs(self.output_dir, exist_ok=True)

context = ExtractContext()

def read_eigen_matrix(var_name, rows, cols):
    """
    直接从内存读取 Eigen 矩阵数据 (ColMajor)
    比循环 dereference 快得多且更稳定
    """
    try:
        val = gdb.parse_and_eval(var_name)
        
        # 尝试获取数据的内存地址
        addr = None
        if val.address:
            addr = val.address
        elif val.type.code == gdb.TYPE_CODE_REF:
            addr = val.cast(val.type.target().pointer())
        elif hasattr(val, 'address'):
            addr = val.address
            
        if not addr:
            # 最后的尝试：如果是 Eigen::Matrix，通常可以用 .data() 方法（如果在调试符号中可用）
            # 或者直接转换数组衰退为指针
            try:
                addr = val.cast(val.type.strip_typedefs()).address
            except:
                pass

        if not addr:
            print(f"[Warn] 无法获取 {var_name} 地址 (可能被优化)")
            return None

        # 将地址转换为 double*
        double_ptr_type = gdb.lookup_type('double').pointer()
        ptr = addr.cast(double_ptr_type)
        
        # 计算需要读取的字节数
        cnt = rows * cols
        size_bytes = cnt * 8 # double = 8 bytes
        
        # 读取内存块
        inferior = gdb.selected_inferior()
        mem = inferior.read_memory(ptr, size_bytes)
        
        # 解析二进制数据
        # 'd' 表示 double
        floats = struct.unpack(f'{cnt}d', mem)
        
        # Eigen 默认为列优先 (Column Major)
        # floats 排列: [col0_row0, col0_row1, ..., col1_row0, ...]
        
        # 我们将其重组为按列组织的列表，方便后续处理
        traj_cols = []
        for c in range(cols):
            start_idx = c * rows
            col_data = floats[start_idx : start_idx + rows]
            traj_cols.append(col_data)
            
        return traj_cols
            
    except Exception as e:
        print(f"[ReadError] 读取矩阵 {var_name} 失败: {e}")
        return None

def save_snapshot(jump_idx, traj_data):
    """将数据保存为 JSON"""
    if not traj_data:
        return None

    try:
        # 重组数据为可视化脚本常用的格式
        # traj_data[col][row]
        cols = len(traj_data)

        try:
            start_idx = int(gdb.parse_and_eval("start_idx"))
            end_idx = int(gdb.parse_and_eval("end_idx"))
        except Exception:
            start_idx = -1
            end_idx = -1
        
        traj_dict = {
            'yaw':       [traj_data[i][0] for i in range(cols)],
            'yaw_vel':   [traj_data[i][1] for i in range(cols)],
            'pitch':     [traj_data[i][2] for i in range(cols)],
            'pitch_vel': [traj_data[i][3] for i in range(cols)],
        }
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = f"{context.output_dir}/traj_{timestamp}_{context.counter:04d}.json"
        
        output = {
            'jump_idx': int(jump_idx),
            'start_idx': start_idx,
            'end_idx': end_idx, 
            'traj': traj_dict,
            'timestamp': timestamp,
            'frame': context.counter
        }
        
        with open(filename, 'w') as f:
            json.dump(output, f, indent=2)
            
        context.counter += 1
        return filename
    except Exception as e:
        print(f"[SaveError] {e}")
        return None

def save_snapshot_withruckig(jump_idx, traj_data, ruckig_data):
    """将数据保存为 JSON"""
    if not traj_data:
        return None

    try:
        # 重组数据为可视化脚本常用的格式
        # traj_data[col][row]
        cols = len(traj_data)

        try:
            start_idx = int(gdb.parse_and_eval("start_idx"))
            end_idx = int(gdb.parse_and_eval("end_idx"))
        except Exception:
            start_idx = -1
            end_idx = -1
        
        traj_dict = {
            'yaw':       [traj_data[i][0] for i in range(cols)],
            'yaw_vel':   [traj_data[i][1] for i in range(cols)],
            'pitch':     [traj_data[i][2] for i in range(cols)],
            'pitch_vel': [traj_data[i][3] for i in range(cols)],
        }
        ruckig_dict = None
        if ruckig_data and isinstance(ruckig_data, list):
            r_cols = len(ruckig_data)
            # 每行期望 [yaw, yaw_vel, pitch, pitch_vel]
            ruckig_dict = {
                'yaw':       [ruckig_data[i][0] if i < r_cols else 0.0 for i in range(cols)],
                'yaw_vel':   [ruckig_data[i][1] if i < r_cols else 0.0 for i in range(cols)],
                'pitch':     [ruckig_data[i][2] if i < r_cols else 0.0 for i in range(cols)],
                'pitch_vel': [ruckig_data[i][3] if i < r_cols else 0.0 for i in range(cols)],
            }
        else:
            ruckig_dict = {}
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = f"{context.output_dir}/traj_{timestamp}_{context.counter:04d}.json"
        
        output = {
            'jump_idx': int(jump_idx),
            'start_idx': start_idx,
            'end_idx': end_idx, 
            'traj': traj_dict,
            'ruckig': ruckig_dict,
            'timestamp': timestamp,
            'frame': context.counter
        }
        
        with open(filename, 'w') as f:
            json.dump(output, f, indent=2)
            
        context.counter += 1
        return filename
    except Exception as e:
        print(f"[SaveError] {e}")
        return None

# 定义自动断点类
class AutoExtractBreakpoint(gdb.Breakpoint):
    def stop(self):
        """断点触发时的回调"""
        try:
            # 1. 尝试获取上下文变量
            try:
                jump_idx = gdb.parse_and_eval("jump_idx")
                jump_idx = int(jump_idx)
            except:
                # 可能是优化问题，或者还没运行到定义处，或者在其他线程
                return False 

            # 2. 读取矩阵
            traj_data = read_eigen_matrix("traj", context.rows, context.horizon)
            ruckig_data = read_eigen_matrix("ruckig_vis_traj", 4, 100) # 这里假设 HORIZON=100
            if not traj_data:
                print("[AutoExtract] 未找到 traj，尝试读取 traj...")
                traj_data = read_eigen_matrix("traj", context.rows, context.horizon)
            if not ruckig_data:
                print("[AutoExtract] 未找到 ruckig_vis_traj，尝试读取 ruckig_vis_traj...")
                ruckig_data = read_eigen_matrix("ruckig_vis_traj", context.rows, context.horizon)
            
            # 3. 保存
            if traj_data and ruckig_data:
                save_snapshot_withruckig(jump_idx, traj_data,ruckig_data)
            
        except Exception as e:
            print(f"[AutoExtract] 遇到异常: {e}")
        
        # 返回 False 表示 **不停止** 程序运行，继续执行
        return False

class TrajToolCommand(gdb.Command):
    """
    轨迹提取工具集
    用法:
      traj_tool start [location] : 开启自动提取 (默认 planner.cpp:1564)
      traj_tool stop             : 停止自动提取
      traj_tool once             : 手动提取当前状态
    """
    
    def __init__(self):
        super(TrajToolCommand, self).__init__("traj_tool", gdb.COMMAND_USER)
        
    def invoke(self, arg, from_tty):
        args = arg.split()
        if not args:
            print("请指定子命令: start, stop, once")
            print("示例: traj_tool start planner.cpp:1357")
            return
            
        cmd = args[0]
        
        if cmd == "start":
            # 清理旧断点
            if context.bp:
                try:
                    context.bp.delete()
                except:
                    pass
            
            # 默认位置，用户可覆盖
            loc = args[1] if len(args) > 1 else "planner.cpp:1357"
            
            try:
                context.bp = AutoExtractBreakpoint(loc)
                print(f"[traj_tool] 自动提取已开启。")
                print(f"            断点位置: {loc}")
                print(f"            输出目录: {context.output_dir}")
                print(f"            程序将在断点处自动记录并继续运行。")
                print(f"注意：请确保断点位置处存在 'traj' 和 'jump_idx' 局部变量。")
            except Exception as e:
                print(f"[traj_tool] 设置断点失败: {e}")
                
        elif cmd == "stop":
            if context.bp:
                context.bp.delete()
                context.bp = None
                print("[traj_tool] 自动提取已停止 (断点已移除)。")
            else:
                print("[traj_tool] 当前没有活动的自动提取断点。")
                
        elif cmd == "once":
            try:
                jump_idx = gdb.parse_and_eval("jump_idx")
                traj_data = read_eigen_matrix("traj", context.rows, context.horizon)
                ruckig_data = read_eigen_matrix("ruckig_vis_traj", context.rows, context.horizon)
                if traj_data:
                    saved = save_snapshot_withruckig(jump_idx, traj_data,ruckig_data)
                    print(f"[traj_tool] 手动提取成功: {saved}")
            except Exception as e:
                print(f"[traj_tool] 提取失败 (请确认当前上下文包含 'traj' 和 'jump_idx'): {e}")
        else:
            print(f"未知命令: {cmd}")

# 注册命令
TrajToolCommand()
print("[GDB Script] 轨迹提取工具 v2.0 已加载")
print("命令: traj_tool start | stop | once")