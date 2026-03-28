#!/bin/bash

# 确保脚本发生错误时退出
set -e

# 定义要运行的可执行文件和参数
EXECUTABLE="./build/Debug/planner_test_offline"
ARGS="-d=3.0 -w=16.0 -v=0.0 configs/standard3.yaml"
PYTHON_SCRIPT="tools/other/gdb_extract_traj.py"

# 检查可执行文件是否存在
if [ ! -f "$EXECUTABLE" ]; then
    echo "Error: Executable $EXECUTABLE not found!"
    exit 1
fi

# 启动 GDB
# -ex "source ..." : 加载 python 脚本
# -ex "traj_tool start" : 执行 python 脚本中定义的命令 (假设 traj_tool 是注册的命令)
# -ex "run" : 开始运行程序
# --args : 后面紧跟可执行文件及其参数

echo "Starting GDB..."
gdb -ex "source $PYTHON_SCRIPT" \
    -ex "traj_tool start planner.cpp:1352" \
    -ex "run" \
    --args $EXECUTABLE $ARGS