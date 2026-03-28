# sleep 5
# cd /home/wdr/wdr_26_sp_3/wdr_26_sp_2-main/
# screen \
#     # -L \
#     # -Logfile logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog \
#     # -d \
#     # -m \
#     bash -c "pkill -9 rw_tracker_test"
#     bash -c "./build/rw_tracker_test"


    
# !/bin/bash

# 延迟启动，确保系统初始化完成
sleep 15

cd /home/wdr/wdr_26_sp_3/wdr_26_sp_2-main/ || exit 1

PROG_NAME="rw_tracker_test"

# 新增标志变量，记录是否为开机首次启动
FIRST_RUN=true

while true; do
    timestamp=$(date "+%Y-%m-%d_%H-%M-%S")

    echo "[$timestamp] 清理旧进程..."

    # 杀死旧进程
    pkill -f "./build/$PROG_NAME" 2>/dev/null
    sleep 1

    echo "[$timestamp] 启动 $PROG_NAME"

    # 将输出重定向到 /dev/null 丢弃，不再记录到日志文件
    ./build/$PROG_NAME > /dev/null 2>&1 &

    pid=$!

    # 如果是开机第一次启动，等待几秒后将其杀死
    if [ "$FIRST_RUN" = true ]; then
        echo "[$timestamp] 首次启动标记触发，等待 3 秒后杀死 $PROG_NAME 进程..."
        sleep 3  # 可以根据程序初始化的实际耗时稍微调整等待时间
        kill -9 $pid 2>/dev/null
        # 将标志位置为 false，确保后续循环不再主动去杀进程
        FIRST_RUN=false
    fi

    # 等待程序结束（如果第一次已被杀掉，这里会直接退出等待；后续则起保护和持续等待作用）
    wait $pid

    echo "[$timestamp] 程序退出，5秒后重启..."

    sleep 5
done