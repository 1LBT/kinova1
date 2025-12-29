#!/bin/bash

# Kinova机械臂抓取程序测试脚本
# 用于快速启动和测试修改后的功能

echo "=========================================="
echo "Kinova机械臂抓取程序测试脚本"
echo "=========================================="

# 检查ROS环境
if [ -z "$ROS_PACKAGE_PATH" ]; then
    echo "错误: ROS环境未设置，请先运行 source devel/setup.bash"
    exit 1
fi

# 检查工作空间
if [ ! -f "devel/setup.bash" ]; then
    echo "错误: 请在kinova_ws根目录下运行此脚本"
    exit 1
fi

echo "ROS环境检查通过"

# 显示菜单
show_menu() {
    echo ""
    echo "请选择要运行的程序:"
    echo "1. 运行修改后的开环抓取程序"
    echo "2. 运行XY位置控制测试程序"
    echo "3. 检查TF树结构"
    echo "4. 查看GGCNN话题"
    echo "5. 显示修改总结"
    echo "6. 退出"
    echo ""
}

# 检查TF树
check_tf_tree() {
    echo "生成TF树图..."
    rosrun tf view_frames
    if [ -f "frames.pdf" ]; then
        echo "TF树图已生成: frames.pdf"
        echo "可以使用以下命令查看:"
        echo "evince frames.pdf"
    else
        echo "TF树图生成失败"
    fi
}

# 查看话题
check_topics() {
    echo "GGCNN相关话题:"
    rostopic list | grep -E "ggcnn|pose|position|camera" || echo "未找到相关话题"
    echo ""
    echo "机械臂相关话题:"
    rostopic list | grep -E "m1n6s200|kinova" || echo "未找到机械臂话题"
}

# 显示修改总结
show_summary() {
    if [ -f "src/kinova-ros/ggcnn/ggcnn_kinova_grasping/MODIFICATION_SUMMARY.md" ]; then
        echo "修改总结:"
        echo "=========================================="
        head -50 src/kinova-ros/ggcnn/ggcnn_kinova_grasping/MODIFICATION_SUMMARY.md
        echo "=========================================="
        echo "完整内容请查看: MODIFICATION_SUMMARY.md"
    else
        echo "未找到修改总结文件"
    fi
}

# 主循环
while true; do
    show_menu
    read -p "请输入选择 (1-6): " choice
    
    case $choice in
        1)
            echo "启动修改后的开环抓取程序..."
            echo "注意: 请确保机械臂和相机已正确启动"
            echo "按Ctrl+C可以随时停止程序"
            sleep 2
            rosrun ggcnn_kinova_grasping kinova_open_loop_grasp.py
            ;;
        2)
            echo "启动XY位置控制测试程序..."
            echo "此程序用于测试机械臂的XY位置控制功能"
            sleep 2
            rosrun ggcnn_kinova_grasping test_xy_control.py
            ;;
        3)
            check_tf_tree
            ;;
        4)
            check_topics
            ;;
        5)
            show_summary
            ;;
        6)
            echo "退出测试脚本"
            exit 0
            ;;
        *)
            echo "无效选择，请重新输入"
            ;;
    esac
    
    echo ""
    read -p "按Enter继续..."
done