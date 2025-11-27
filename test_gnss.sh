#!/bin/bash
# 一键测试脚本

echo "============================================"
echo "  GNSS Converter 一键测试"
echo "============================================"
echo ""
echo "此脚本将："
echo "  1. 检查并安装依赖（socat）"
echo "  2. 创建虚拟串口"
echo "  3. 启动 GNSS Converter"
echo "  4. 发送模拟 GPS 数据"
echo "  5. 验证输出"
echo ""
read -p "按 Enter 继续，或 Ctrl+C 取消..."
echo ""

# 检查并安装 socat
if ! command -v socat &> /dev/null; then
    echo "→ 安装 socat..."
    sudo apt-get install -y socat
    if [ $? -ne 0 ]; then
        echo "❌ socat 安装失败"
        echo "请手动运行: sudo apt-get install socat"
        exit 1
    fi
fi
echo "✅ socat 已就绪"

# 检查 pyserial
if ! python3 -c "import serial" 2>/dev/null; then
    echo "→ 安装 pyserial..."
    pip3 install pyserial --user
fi
echo "✅ pyserial 已就绪"
echo ""

# 执行测试
cd /home/cplus/Desktop/yunle_sensors
bash src/gnss_converter/scripts/quick_test.sh
