#!/usr/bin/env python3
"""
可视化轨迹数据，显示 traj 和 jump_idx 的位置
使用方法：python3 visualize_traj.py <json_file> 或 python3 visualize_traj.py --watch
"""

import json
import sys
import os
import glob
import time
import argparse
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle

def load_traj_data(json_file):
    """加载轨迹数据"""
    with open(json_file, 'r') as f:
        data = json.load(f)
    return data

def visualize_traj(data, save_path=None, block=True, title_suffix=""):
    """可视化轨迹数据"""
    traj = data['traj']
    ruckig = data['ruckig']
    jump_idx = data['jump_idx']

    start_idx = data.get('start_idx', -1)
    end_idx = data.get('end_idx', -1)

    frame = data.get('frame', 0)
    
    HORIZON = len(traj['yaw'])
    DT = 0.01
    time_axis = np.arange(HORIZON) * DT
    
    # 创建图形
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(f'轨迹可视化 - Frame {frame} | jump_idx = {jump_idx} {title_suffix}', fontsize=14)
    
    # 辅助绘图函数
    def plot_sub(ax, data_key, title, ylabel, color):
        ax.plot(time_axis, traj[data_key], color=color, linewidth=1.5, label=data_key)
        if ruckig and data_key in ruckig:
            r_data = ruckig[data_key]
            # 计算偏移量：如果 start_idx 有效，从该时间点开始绘制
            offset_idx = max(0, start_idx)
            # 构建 Ruckig 专用的时间轴
            r_time_axis = (np.arange(len(r_data)) + offset_idx) * DT
            
            # 过滤掉超出显示范围的部分(可选)
            valid_mask = r_time_axis < (HORIZON * DT + 0.5) 
            
            ax.plot(r_time_axis[valid_mask], np.array(r_data)[valid_mask], 
                    color='darkorange', linestyle='--', linewidth=2.5, label='Ruckig')

        if jump_idx >= 0 and jump_idx < HORIZON:
            ax.axvline(x=time_axis[jump_idx], color='r', linestyle='--', linewidth=2, label=f'Jump (idx={jump_idx})')
            ax.plot(time_axis[jump_idx], traj[data_key][jump_idx], 'ro', markersize=8)

            # [新增] 绘制 Ruckig 起点 (绿色)
        if start_idx >= 0 and start_idx < HORIZON:
            ax.axvline(x=time_axis[start_idx], color='g', linestyle=':', linewidth=2, label=f'Start ({start_idx})')
            ax.plot(time_axis[start_idx], traj[data_key][start_idx], 'g^', markersize=8)

        # [新增] 绘制 Ruckig 终点 (紫色)
        if end_idx >= 0 and end_idx < HORIZON:
            ax.axvline(x=time_axis[end_idx], color='m', linestyle=':', linewidth=2, label=f'End ({end_idx})')
            ax.plot(time_axis[end_idx], traj[data_key][end_idx], 'mv', markersize=8)
            
        ax.set_xlabel('时间 (s)')
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        ax.legend()

    # 1. Yaw 位置
    plot_sub(axes[0, 0], 'yaw', 'Yaw 位置 (rad)', 'Yaw', 'b')
    
    # 2. Yaw 速度
    plot_sub(axes[0, 1], 'yaw_vel', 'Yaw 速度 (rad/s)', 'Yaw Vel', 'g')
    
    # 3. Pitch 位置
    plot_sub(axes[1, 0], 'pitch', 'Pitch 位置 (rad)', 'Pitch', 'm')
    
    # 4. Pitch 速度
    plot_sub(axes[1, 1], 'pitch_vel', 'Pitch 速度 (rad/s)', 'Pitch Vel', 'c')
    
    # 在右下角添加详细信息框
    if jump_idx >= 0 and jump_idx < HORIZON:
        info_text = f"Frame: {frame}\n"
        info_text += f"阶跃点信息:\n"
        info_text += f"  索引: {jump_idx}\n"
        info_text += f"  时间: {time_axis[jump_idx]:.3f}s\n"
        if jump_idx > 0:
            yaw_diff = traj['yaw'][jump_idx] - traj['yaw'][jump_idx-1]
            vel_diff = traj['yaw_vel'][jump_idx] - traj['yaw_vel'][jump_idx-1]
            info_text += f"  dYaw: {yaw_diff:.4f}\n"
            info_text += f"  dVel: {vel_diff:.4f}"
        
        # 将文本放置在图表右下角的空白区域（使用 fig 坐标）
        fig.text(0.98, 0.02, info_text, fontsize=10, 
                horizontalalignment='right', 
                verticalalignment='bottom', 
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    plt.subplots_adjust(top=0.92) # 留出标题空间
    
    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"已保存图像到: {save_path}")
        plt.close(fig)
    else:
        if block:
            plt.show()
        else:
            plt.draw()
            plt.pause(0.1) 

def watch_mode(output_dir="/tmp/gdb_traj_data"):
    """监视模式：自动检测新文件并可视化"""
    print(f"监视模式：监听 {output_dir}")
    print("提示：按 Ctrl+C 退出")
    
    processed_files = set()
    
    # 初始扫描，标记现有文件，避免一开始弹出一堆窗口
    existing_files = glob.glob(os.path.join(output_dir, "traj_*.json"))
    for f in existing_files:
        processed_files.add(f)
    print(f"已忽略启动前的 {len(existing_files)} 个历史文件。等待新文件...")

    try:
        while True:
            # 查找所有 JSON 文件
            json_files = glob.glob(os.path.join(output_dir, "traj_*.json"))
            # 按修改时间排序
            json_files.sort(key=os.path.getmtime)
            
            for json_file in json_files:
                if json_file not in processed_files:
                    print(f"\n[NEW] 检测到新轨迹: {os.path.basename(json_file)}")
                    try:
                        data = load_traj_data(json_file)
                        # 在监视模式下，我们通常希望非阻塞显示，或者每次只显示最新的一张
                        # 这里使用 plt.show(block=False) 的类似效果，但为了防止窗口过多，
                        # Python 的 matplotlib 处理多进程 GUI 事件循环比较麻烦。
                        # 简单起见，这里每次弹窗会阻塞，直到用户关闭窗口才继续监听下一个。
                        # 如果需要实时流式查看，建议修改为单窗口刷新模式。
                        visualize_traj(data, block=True, title_suffix=f"[{os.path.basename(json_file)}]") 
                        processed_files.add(json_file)
                    except Exception as e:
                        print(f"处理文件时出错: {e}")
            
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n停止监视。")

def main():
    parser = argparse.ArgumentParser(description='可视化轨迹数据')
    parser.add_argument('json_file', nargs='?', help='JSON 数据文件路径')
    parser.add_argument('--watch', action='store_true', help='监视模式：自动检测新产生的文件')
    parser.add_argument('--save', type=str, help='保存图像路径（不显示）')
    parser.add_argument('--dir', type=str, default='/tmp/gdb_traj_data', help='数据目录（监视/默认查找目录）')
    
    args = parser.parse_args()
    
    if args.watch:
        watch_mode(args.dir)
    elif args.json_file:
        data = load_traj_data(args.json_file)
        visualize_traj(data, args.save)
    else:
        # 如果没有指定文件，查找最新的文件
        json_files = glob.glob(os.path.join(args.dir, "traj_*.json"))
        if json_files:
            latest_file = max(json_files, key=os.path.getmtime)
            print(f"使用最新文件: {latest_file}")
            data = load_traj_data(latest_file)
            visualize_traj(data, args.save, title_suffix=f"[{os.path.basename(latest_file)}]")
        else:
            print("错误：未找到数据文件")
            print(f"搜索目录: {args.dir}")
            sys.exit(1)

if __name__ == '__main__':
    main()