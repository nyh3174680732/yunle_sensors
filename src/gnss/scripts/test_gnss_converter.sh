#!/bin/bash

# GNSS Converter 测试脚本（改进版）

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 虚拟串口路径
VGPS_WRITER="/tmp/ttyVGPS0"
VGPS_READER="/tmp/ttyVGPS1"

# 全局进程 PID
SOCAT_PID=""
CONVERTER_PID=""
SIMULATOR_PID=""

# ========================================
# 清理函数
# ========================================
cleanup() {
    echo ""
    echo -e "${YELLOW}→ 停止所有进程...${NC}"
    
    # 1. 先停止模拟器（停止发送数据）
    if [ ! -z "$SIMULATOR_PID" ]; then
        echo -e "${YELLOW}  停止 GNSS 模拟器 (PID: $SIMULATOR_PID)${NC}"
        kill -TERM $SIMULATOR_PID 2>/dev/null || true
        wait $SIMULATOR_PID 2>/dev/null || true
    fi
    
    # 2. 再停止转换节点（等待它处理完剩余数据）
    if [ ! -z "$CONVERTER_PID" ]; then
        echo -e "${YELLOW}  停止 GNSS Converter (PID: $CONVERTER_PID)${NC}"
        kill -TERM $CONVERTER_PID 2>/dev/null || true
        
        # 等待最多 3 秒让节点正常退出
        for i in {1..30}; do
            if ! kill -0 $CONVERTER_PID 2>/dev/null; then
                break
            fi
            sleep 0.1
        done
        
        # 如果还没退出，强制杀死
        if kill -0 $CONVERTER_PID 2>/dev/null; then
            echo -e "${YELLOW}  强制停止 GNSS Converter${NC}"
            kill -KILL $CONVERTER_PID 2>/dev/null || true
        fi
    fi
    
    # 3. 最后停止 socat（关闭虚拟串口）
    if [ ! -z "$SOCAT_PID" ]; then
        echo -e "${YELLOW}  停止 socat (PID: $SOCAT_PID)${NC}"
        kill -TERM $SOCAT_PID 2>/dev/null || true
        wait $SOCAT_PID 2>/dev/null || true
    fi
    
    # 4. 清理虚拟串口文件
    echo -e "${YELLOW}  清理虚拟串口文件${NC}"
    rm -f ${VGPS_WRITER} ${VGPS_READER}
    
    # 5. 确保所有相关进程都被清理
    pkill -f "simulate_gnss.py" 2>/dev/null || true
    pkill -f "gnss_driver_node" 2>/dev/null || true
    pkill -f "socat.*ttyVGPS" 2>/dev/null || true
    
    echo -e "${GREEN}✓ 清理完成${NC}"
    echo ""
    echo -e "${GREEN}============================================${NC}"
    echo -e "${GREEN}    测试完成${NC}"
    echo -e "${GREEN}============================================${NC}"
}

# 捕获退出信号
trap cleanup EXIT INT TERM

echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}    GNSS Converter 测试脚本${NC}"
echo -e "${BLUE}============================================${NC}"
echo ""

# ========================================
# 检查依赖
# ========================================
if ! command -v socat &> /dev/null; then
    echo -e "${YELLOW}⚠ socat 未安装，正在安装...${NC}"
    sudo apt-get update
    sudo apt-get install -y socat
fi

if ! python3 -c "import serial" 2>/dev/null; then
    echo -e "${YELLOW}⚠ pyserial 未安装，正在安装...${NC}"
    pip3 install pyserial
fi

# ========================================
# 清理旧资源
# ========================================
echo -e "${YELLOW}→ 清理旧的虚拟串口...${NC}"
rm -f ${VGPS_WRITER} ${VGPS_READER}
pkill -f "socat.*ttyVGPS" 2>/dev/null || true
pkill -f "simulate_gnss.py" 2>/dev/null || true
pkill -f "gnss_driver_node" 2>/dev/null || true
sleep 1

# ========================================
# 创建虚拟串口
# ========================================
echo -e "${GREEN}✓ 创建虚拟串口对...${NC}"
echo -e "  写入端: ${VGPS_WRITER}"
echo -e "  读取端: ${VGPS_READER}"

socat -d -d pty,raw,echo=0,link=${VGPS_WRITER} pty,raw,echo=0,link=${VGPS_READER} &
SOCAT_PID=$!

sleep 2

if [ ! -e ${VGPS_WRITER} ] || [ ! -e ${VGPS_READER} ]; then
    echo -e "${RED}✗ 虚拟串口创建失败${NC}"
    exit 1
fi

chmod 666 ${VGPS_WRITER} ${VGPS_READER}
echo -e "${GREEN}✓ 虚拟串口创建成功${NC}"
echo ""

# ========================================
# 启动 ROS2 节点
# ========================================
echo -e "${YELLOW}→ 加载 ROS2 环境...${NC}"
source /home/cplus/Desktop/yunle_sensors/install/setup.bash
echo -e "${GREEN}✓ ROS2 环境已加载${NC}"
echo ""

echo -e "${YELLOW}→ 启动 GNSS Converter 节点...${NC}"
ros2 run gnss gnss_driver_node --ros-args \
    -p serial_port:=${VGPS_READER} \
    -p baudrate:=115200 &
CONVERTER_PID=$!

sleep 3

if ! kill -0 $CONVERTER_PID 2>/dev/null; then
    echo -e "${RED}✗ GNSS Converter 节点启动失败${NC}"
    exit 1
fi

echo -e "${GREEN}✓ GNSS Converter 节点已启动 (PID: ${CONVERTER_PID})${NC}"
echo ""

# ========================================
# 启动模拟器
# ========================================
echo -e "${YELLOW}→ 启动 GNSS 数据模拟器...${NC}"
python3 /home/cplus/Desktop/yunle_sensors/src/gnss/scripts/simulate_gnss.py \
    ${VGPS_WRITER} 115200 1.0 &
SIMULATOR_PID=$!

sleep 2

if ! kill -0 $SIMULATOR_PID 2>/dev/null; then
    echo -e "${RED}✗ GNSS 数据模拟器启动失败${NC}"
    exit 1
fi

echo -e "${GREEN}✓ GNSS 数据模拟器已启动 (PID: ${SIMULATOR_PID})${NC}"
echo ""

# ========================================
# 等待数据处理
# ========================================
echo -e "${BLUE}→ 等待数据处理...${NC}"
sleep 5

# ========================================
# 检查话题数据
# ========================================
echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}    检查 ROS2 话题数据${NC}"
echo -e "${BLUE}============================================${NC}"
echo ""

if ros2 topic list | grep -q "/sensing/gnss/pose_with_covariance"; then
    echo -e "${GREEN}✓ 话题 /sensing/gnss/pose_with_covariance 存在${NC}"
    echo ""

    echo -e "${YELLOW}→ 话题信息:${NC}"
    ros2 topic info /sensing/gnss/pose_with_covariance
    echo ""

    echo -e "${YELLOW}→ 话题频率:${NC}"
    timeout 5 ros2 topic hz /sensing/gnss/pose_with_covariance || true
    echo ""

    echo -e "${YELLOW}→ 接收到的消息示例:${NC}"
    echo -e "${BLUE}--------------------------------------------${NC}"
    timeout 5 ros2 topic echo /sensing/gnss/pose_with_covariance --once || true
    echo -e "${BLUE}--------------------------------------------${NC}"
    echo ""

    echo -e "${GREEN}✓✓✓ 测试成功！GNSS Converter 正常工作！${NC}"
else
    echo -e "${RED}✗ 话题 /sensing/gnss/pose_with_covariance 不存在${NC}"
    echo -e "${RED}✗ 测试失败${NC}"
fi

echo ""
echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}    测试选项${NC}"
echo -e "${BLUE}============================================${NC}"
echo ""

# ========================================
# 询问是否继续运行
# ========================================
echo -e "${YELLOW}是否保持节点运行以继续观察？(y/n, 默认10秒后自动退出)${NC}"

# 使用 read 的超时功能
if read -t 10 -n 1 KEEP_RUNNING; then
    echo ""
else
    KEEP_RUNNING="n"
    echo ""
    echo -e "${YELLOW}超时，自动退出...${NC}"
fi

if [ "$KEEP_RUNNING" = "y" ] || [ "$KEEP_RUNNING" = "Y" ]; then
    echo -e "${GREEN}✓ 节点继续运行...${NC}"
    echo ""
    echo -e "${YELLOW}→ 使用以下命令查看实时数据:${NC}"
    echo -e "  ${BLUE}ros2 topic echo /sensing/gnss/pose_with_covariance${NC}"
    echo ""
    echo -e "${YELLOW}→ 按 Ctrl+C 停止所有进程${NC}"
    echo ""

    # 等待用户中断（cleanup 会在 EXIT 时自动调用）
    wait
else
    echo -e "${YELLOW}→ 准备退出...${NC}"
    # cleanup 会在 EXIT 时自动调用
fi
