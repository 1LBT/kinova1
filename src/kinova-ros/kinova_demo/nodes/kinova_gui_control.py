#!/usr/bin/env python3
"""
Kinova机械臂可视化控制面板
基于PyQt5和ROS开发
"""
import sys
import rospy
import threading
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QGroupBox, QSlider, QDoubleSpinBox, QLabel, QPushButton,
                             QLineEdit, QComboBox, QTabWidget, QGridLayout, QMessageBox, QRadioButton)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPalette, QColor

import actionlib
import kinova_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
from kinova_msgs.srv import *
from sensor_msgs.msg import JointState


class KinovaGUI(QMainWindow):
    """Kinova机械臂可视化控制界面"""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Kinova机械臂可视化控制面板")
        self.setGeometry(100, 100, 1000, 800)
        
        # 机器人参数
        self.robot_prefix = "m1n6s200_"
        self.nb_joints = 6
        self.current_joint_positions = [0.0] * 7
        self.current_cartesian_pose = [0.0] * 7  # x, y, z, qx, qy, qz, qw
        self.current_gripper_positions = [0.0] * 3
        self.subscriber = None  # 添加这一行来保存订阅者引用
        
        # 控制模式
        self.control_mode = "position"  # position, velocity, torque
        
        # 创建主控件
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        
        # 创建布局
        self.main_layout = QVBoxLayout(self.central_widget)
        
        # 创建标签页
        self.tab_widget = QTabWidget()
        self.main_layout.addWidget(self.tab_widget)
        
        # 创建各个标签页
        self.create_connection_tab()
        self.create_joint_control_tab()
        self.create_cartesian_control_tab()
        self.create_gripper_control_tab()
        self.create_predefined_actions_tab()
        self.create_advanced_control_tab()
        self.create_status_tab()
        
        # 启动ROS订阅
        self.start_ros_subscribers()
        
        # 启动定时更新
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_gui)
        self.update_timer.start(100)  # 100ms更新一次
    
    def create_connection_tab(self):
        """创建连接设置标签页"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 机器人类型选择
        robot_group = QGroupBox("机器人类型设置")
        robot_layout = QGridLayout(robot_group)
        
        robot_layout.addWidget(QLabel("机器人型号:"), 0, 0)
        self.robot_type_combo = QComboBox()
        self.robot_type_combo.addItems(["m1n6s200"])
        self.robot_type_combo.currentTextChanged.connect(self.on_robot_type_changed)
        robot_layout.addWidget(self.robot_type_combo, 0, 1)
        
        robot_layout.addWidget(QLabel("关节数量:"), 1, 0)
        self.joint_count_label = QLabel(str(self.nb_joints))
        robot_layout.addWidget(self.joint_count_label, 1, 1)
        
        robot_layout.addWidget(QLabel("前缀:"), 2, 0)
        self.prefix_label = QLabel(self.robot_prefix)
        robot_layout.addWidget(self.prefix_label, 2, 1)
        
        # 在create_connection_tab方法中添加
        simulation_group = QGroupBox("连接模式")
        simulation_layout = QHBoxLayout(simulation_group)
        
        self.real_robot_radio = QRadioButton("真实机器人")
        self.real_robot_radio.setChecked(True)
        self.simulation_radio = QRadioButton("仿真环境")
        
        simulation_layout.addWidget(self.real_robot_radio)
        simulation_layout.addWidget(self.simulation_radio)
        layout.addWidget(simulation_group)
        
        # 连接控制按钮
        button_group = QHBoxLayout()
        
        self.connect_button = QPushButton("连接机器人")
        self.connect_button.clicked.connect(self.connect_robot)
        button_group.addWidget(self.connect_button)
        
        self.disconnect_button = QPushButton("断开连接")
        self.disconnect_button.clicked.connect(self.disconnect_robot)
        button_group.addWidget(self.disconnect_button)
        
        layout.addLayout(button_group)
        
        self.tab_widget.addTab(tab, "连接设置")
    
    def create_joint_control_tab(self):
        """创建关节控制标签页"""  # 这里需要缩进，与下面的代码保持一致
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 关节控制组
        joint_group = QGroupBox("关节角度控制")
        joint_layout = QGridLayout(joint_group)
        
        self.joint_sliders = []
        self.joint_spinboxes = []
        
        for i in range(7):
            # 滑块
            slider = QSlider(Qt.Horizontal)
            slider.setRange(-180, 180)
            slider.setValue(0)
            slider.valueChanged.connect(lambda value, joint=i: self.on_joint_slider_changed(joint, value))
            self.joint_sliders.append(slider)
            
            # 数值输入框
            spinbox = QDoubleSpinBox()
            spinbox.setRange(-180.0, 180.0)
            spinbox.setValue(0.0)
            spinbox.setSingleStep(0.1)
            spinbox.valueChanged.connect(lambda value, joint=i: self.on_joint_spinbox_changed(joint, value))
            self.joint_spinboxes.append(spinbox)
            
            # 布局
            joint_layout.addWidget(QLabel(f"关节 {i+1}:"), i, 0)
            joint_layout.addWidget(slider, i, 1)
            joint_layout.addWidget(spinbox, i, 2)
        
        layout.addWidget(joint_group)
        
        # 控制按钮
        control_layout = QHBoxLayout()
        
        self.send_joint_button = QPushButton("发送关节角度")
        self.send_joint_button.clicked.connect(self.send_joint_positions)
        control_layout.addWidget(self.send_joint_button)
        
        self.reset_joints_button = QPushButton("重置为零")
        self.reset_joints_button.clicked.connect(self.reset_joints)
        control_layout.addWidget(self.reset_joints_button)
        
        layout.addLayout(control_layout)
        
        self.tab_widget.addTab(tab, "关节控制")
    
    def create_cartesian_control_tab(self):
        """创建笛卡尔控制标签页"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 位置控制组
        position_group = QGroupBox("位置控制 (米)")
        position_layout = QGridLayout(position_group)
        
        self.position_spinboxes = []
        position_labels = ["X", "Y", "Z"]
        
        for i, label in enumerate(position_labels):
            spinbox = QDoubleSpinBox()
            spinbox.setRange(-1.0, 1.0)
            spinbox.setValue(0.0)
            spinbox.setSingleStep(0.01)
            self.position_spinboxes.append(spinbox)
            
            position_layout.addWidget(QLabel(f"{label}:"), i, 0)
            position_layout.addWidget(spinbox, i, 1)
        
        layout.addWidget(position_group)
        
        # 姿态控制组
        orientation_group = QGroupBox("姿态控制 (四元数)")
        orientation_layout = QGridLayout(orientation_group)
        
        self.orientation_spinboxes = []
        orientation_labels = ["QX", "QY", "QZ", "QW"]
        
        for i, label in enumerate(orientation_labels):
            spinbox = QDoubleSpinBox()
            spinbox.setRange(-1.0, 1.0)
            spinbox.setValue(0.0 if i < 3 else 1.0)
            spinbox.setSingleStep(0.01)
            self.orientation_spinboxes.append(spinbox)
            
            orientation_layout.addWidget(QLabel(f"{label}:"), i, 0)
            orientation_layout.addWidget(spinbox, i, 1)
        
        layout.addWidget(orientation_group)
        
        # 控制按钮
        control_layout = QHBoxLayout()
        
        self.send_pose_button = QPushButton("发送位姿")
        self.send_pose_button.clicked.connect(self.send_cartesian_pose)
        control_layout.addWidget(self.send_pose_button)
        
        self.reset_pose_button = QPushButton("重置位姿")
        self.reset_pose_button.clicked.connect(self.reset_pose)
        control_layout.addWidget(self.reset_pose_button)
        
        layout.addLayout(control_layout)
        
        self.tab_widget.addTab(tab, "笛卡尔控制")
    
    def create_gripper_control_tab(self):
        """创建夹爪控制标签页"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 夹爪控制组
        gripper_group = QGroupBox("夹爪控制")
        gripper_layout = QGridLayout(gripper_group)
        
        # 整体控制滑块
        gripper_layout.addWidget(QLabel("夹爪开合度:"), 0, 0)
        self.gripper_slider = QSlider(Qt.Horizontal)
        self.gripper_slider.setRange(0, 100)
        self.gripper_slider.setValue(0)  # 这行需要缩进，与上面的行保持一致
        self.gripper_slider.valueChanged.connect(self.on_gripper_slider_changed)
        gripper_layout.addWidget(self.gripper_slider, 0, 1)
        
        self.gripper_spinbox = QDoubleSpinBox()
        self.gripper_spinbox.setRange(0.0, 100.0)
        self.gripper_spinbox.setValue(0.0)
        self.gripper_spinbox.setSingleStep(1.0)
        self.gripper_spinbox.valueChanged.connect(self.on_gripper_spinbox_changed)
        gripper_layout.addWidget(self.gripper_spinbox, 0, 2)
        
        # 单独控制
        for i in range(3):
            gripper_layout.addWidget(QLabel(f"手指 {i+1}:"), i+1, 0)
            spinbox = QDoubleSpinBox()
            spinbox.setRange(0.0, 100.0)
            spinbox.setValue(0.0)
            spinbox.setSingleStep(1.0)
            gripper_layout.addWidget(spinbox, i+1, 1)
            # 控制界面组件
            self.gripper_spinboxes = []
            
        layout.addWidget(gripper_group)
        
        # 控制按钮
        control_layout = QHBoxLayout()
        
        self.open_gripper_button = QPushButton("打开夹爪")
        self.open_gripper_button.clicked.connect(lambda: self.set_gripper_position(0))
        control_layout.addWidget(self.open_gripper_button)
        
        self.close_gripper_button = QPushButton("关闭夹爪")
        self.close_gripper_button.clicked.connect(lambda: self.set_gripper_position(100))
        control_layout.addWidget(self.close_gripper_button)
        
        self.send_gripper_button = QPushButton("发送夹爪命令")
        self.send_gripper_button.clicked.connect(self.send_gripper_position)
        control_layout.addWidget(self.send_gripper_button)
        
        layout.addLayout(control_layout)
        
        self.tab_widget.addTab(tab, "夹爪控制")
    
    def create_predefined_actions_tab(self):
        """创建预定义动作标签页"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 常用动作组
        actions_group = QGroupBox("常用动作")
        actions_layout = QVBoxLayout(actions_group)
        self.home_button = QPushButton("回到初始位置")
        self.home_button.clicked.connect(self.move_home)
        actions_layout.addWidget(self.home_button)
        
        self.zero_torque_button = QPushButton("零力矩模式")
        self.zero_torque_button.clicked.connect(self.set_zero_torque)
        actions_layout.addWidget(self.zero_torque_button)
        
        self.null_space_button = QPushButton("激活零空间模式")
        self.null_space_button.clicked.connect(self.activate_null_space)
        actions_layout.addWidget(self.null_space_button)
        
        layout.addWidget(actions_group)
        
        self.tab_widget.addTab(tab, "预定义动作")
    
    def create_advanced_control_tab(self):
        """创建高级控制标签页"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 控制模式选择
        mode_group = QGroupBox("控制模式")
        mode_layout = QVBoxLayout(mode_group)
        self.position_mode_button = QPushButton("位置控制")
        self.position_mode_button.setCheckable(True)
        self.position_mode_button.setChecked(True)
        self.position_mode_button.clicked.connect(lambda: self.set_control_mode("position"))
        mode_layout.addWidget(self.position_mode_button)
        
        self.velocity_mode_button = QPushButton("速度控制")
        self.velocity_mode_button.setCheckable(True)
        self.velocity_mode_button.clicked.connect(lambda: self.set_control_mode("velocity"))
        mode_layout.addWidget(self.velocity_mode_button)
        
        self.torque_mode_button = QPushButton("力矩控制")
        self.torque_mode_button.setCheckable(True)
        self.torque_mode_button.clicked.connect(lambda: self.set_control_mode("torque"))
        mode_layout.addWidget(self.torque_mode_button)
        
        # 互斥按钮组
        from PyQt5.QtWidgets import QButtonGroup
        self.mode_button_group = QButtonGroup()
        self.mode_button_group.addButton(self.position_mode_button)
        self.mode_button_group.addButton(self.velocity_mode_button)
        self.mode_button_group.addButton(self.torque_mode_button)
        self.mode_button_group.setExclusive(True)
        
        layout.addWidget(mode_group)
        
        # 速度控制组
        velocity_group = QGroupBox("速度控制")
        velocity_layout = QVBoxLayout(velocity_group)
        
        self.velocity_send_button = QPushButton("发送速度命令")
        self.velocity_send_button.clicked.connect(self.send_velocity_command)
        velocity_layout.addWidget(self.velocity_send_button)
        
        layout.addWidget(velocity_group)
        
        # 力矩控制组
        torque_group = QGroupBox("力矩控制")
        torque_layout = QVBoxLayout(torque_group)
        
        self.torque_send_button = QPushButton("发送力矩命令")
        self.torque_send_button.clicked.connect(self.send_torque_command)
        torque_layout.addWidget(self.torque_send_button)
        
        layout.addWidget(torque_group)
        
        self.tab_widget.addTab(tab, "高级控制")
    
    def create_status_tab(self):
        """创建状态显示标签页"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 关节状态组
        joint_status_group = QGroupBox("关节状态")
        joint_status_layout = QGridLayout(joint_status_group)
        
        self.joint_status_labels = []
        for i in range(7):
            joint_status_layout.addWidget(QLabel(f"关节 {i+1}:"), i, 0)
            label = QLabel("0.0")
            joint_status_layout.addWidget(label, i, 1)
            self.joint_status_labels.append(label)
        
        layout.addWidget(joint_status_group)
        
        # 笛卡尔状态组
        cartesian_status_group = QGroupBox("笛卡尔状态")
        cartesian_status_layout = QGridLayout(cartesian_status_group)
        
        status_labels = ["X", "Y", "Z", "QX", "QY", "QZ", "QW"]
        self.cartesian_status_labels = []
        
        for i, label in enumerate(status_labels):
            cartesian_status_layout.addWidget(QLabel(f"{label}:"), i, 0)
            status_label = QLabel("0.0")
            cartesian_status_layout.addWidget(status_label, i, 1)
            self.cartesian_status_labels.append(status_label)
        
        layout.addWidget(cartesian_status_group)
        
        # 夹爪状态组
        gripper_status_group = QGroupBox("夹爪状态")
        gripper_status_layout = QGridLayout(gripper_status_group)
        
        self.gripper_status_labels = []
        for i in range(3):
            gripper_status_layout.addWidget(QLabel(f"手指 {i+1}:"), i, 0)
            label = QLabel("0.0")
            gripper_status_layout.addWidget(label, i, 1)
            self.gripper_status_labels.append(label)
        
        layout.addWidget(gripper_status_group)
        
        self.tab_widget.addTab(tab, "状态显示")
    
    def on_robot_type_changed(self, robot_type):
        """机器人类型改变时的处理"""
        # 对于仿真环境，根据实际情况决定是否需要下划线前缀
        # 用户反馈仿真环境实际使用带下划线的前缀，如"m1n6s200_joint_states"
        if self.simulation_radio.isChecked():
            self.robot_prefix = robot_type + "_"  # 仿真环境实际也带下划线
        else:
            self.robot_prefix = robot_type + "_"  # 真实机器人带下划线
        self.nb_joints = int(robot_type[3])
        self.joint_count_label.setText(str(self.nb_joints))
        self.prefix_label.setText(self.robot_prefix)
    
    def on_joint_slider_changed(self, joint, value):
        """关节滑块改变时的处理"""
        self.joint_spinboxes[joint].setValue(value)
        self.current_joint_positions[joint] = value
    
    def on_joint_spinbox_changed(self, joint, value):
        """关节数值输入框改变时的处理"""
        self.joint_sliders[joint].setValue(int(value))
        self.current_joint_positions[joint] = value
    
    def on_gripper_slider_changed(self, value):
        """夹爪滑块改变时的处理"""
        self.gripper_spinbox.setValue(value)
        for spinbox in self.gripper_spinboxes:
            spinbox.setValue(value)
    
    def on_gripper_spinbox_changed(self, value):
        """夹爪数值输入框改变时的处理"""
        self.gripper_slider.setValue(int(value))
    
    def send_joint_positions(self):
        """发送关节位置命令"""
        from robot_control_modules import joint_position_client
        
        # 获取当前设置的关节角度
        angles = [spinbox.value() for spinbox in self.joint_spinboxes]
        
        # 创建线程执行ROS命令，避免阻塞GUI
        threading.Thread(target=lambda: joint_position_client(angles, self.robot_prefix)).start()
    
    def reset_joints(self):
        """重置关节为零"""
        for i in range(7):
            self.joint_sliders[i].setValue(0)
            self.joint_spinboxes[i].setValue(0.0)
            self.current_joint_positions[i] = 0.0
    
    def send_velocity_command(self):
        """发送速度命令"""
        from robot_control_modules import publishVelCmd
        
        # 这里需要获取速度值，暂时使用默认值
        velocities = [0.0] * 7  # 7个关节的速度值，默认为0
        duration_sec = 1.0  # 持续时间1秒
        
        # 创建线程执行ROS命令
        threading.Thread(target=lambda: publishVelCmd(velocities, duration_sec, self.robot_prefix)).start()
    
    def send_torque_command(self):
        """发送力矩命令"""
        from robot_control_modules import publishTorqueCmd
        
        # 这里需要获取力矩值，暂时使用默认值
        torques = [0.0] * 7  # 7个关节的力矩值，默认为0
        duration_sec = 1.0  # 持续时间1秒
        
        # 创建线程执行ROS命令
        threading.Thread(target=lambda: publishTorqueCmd(torques, duration_sec, self.robot_prefix)).start()
    
    def send_cartesian_pose(self):
        """发送笛卡尔位姿命令"""
        from robot_control_modules import cartesian_pose_client
        
        # 获取位置和姿态
        position = [spinbox.value() for spinbox in self.position_spinboxes]
        orientation = [spinbox.value() for spinbox in self.orientation_spinboxes]
        
        # 创建线程执行ROS命令
        threading.Thread(target=lambda: cartesian_pose_client(position, orientation, self.robot_prefix)).start()
    
    def reset_pose(self):
        """重置位姿为零"""
        for spinbox in self.position_spinboxes:
            spinbox.setValue(0.0)
        for i, spinbox in enumerate(self.orientation_spinboxes):
            spinbox.setValue(0.0 if i < 3 else 1.0)
    
    def set_gripper_position(self, position):
        """设置夹爪位置"""
        self.gripper_slider.setValue(position)
        self.gripper_spinbox.setValue(float(position))
        for spinbox in self.gripper_spinboxes:
            spinbox.setValue(float(position))
    
    def send_gripper_position(self):
        """发送夹爪命令"""
        from robot_control_modules import gripper_client
        
        # 获取夹爪位置
        positions = [spinbox.value() for spinbox in self.gripper_spinboxes]
        
        # 创建线程执行ROS命令
        threading.Thread(target=lambda: gripper_client(positions, self.robot_prefix)).start()
    
    def move_home(self):
        """移动到初始位置"""
        from robot_control_modules import homeRobot
        threading.Thread(target=lambda: homeRobot(self.robot_prefix)).start()
    
    def set_zero_torque(self):
        """设置零力矩模式"""
        from robot_control_modules import ZeroTorque
        threading.Thread(target=lambda: ZeroTorque(self.robot_prefix)).start()
    
    def activate_null_space(self):
        """激活零空间模式"""
        from robot_control_modules import activateNullSpaceMode
        threading.Thread(target=lambda: activateNullSpaceMode(5.0, self.robot_prefix)).start()
    
    def set_control_mode(self, mode):
        """设置控制模式"""
        self.control_mode = mode
    
    def start_ros_subscribers(self):
        """启动ROS订阅者"""
        # 如果已有订阅者，先停止它
        if hasattr(self, 'subscriber') and self.subscriber is not None:
            self.subscriber.unregister()
        
        if self.simulation_radio.isChecked():
            # 仿真环境使用标准关节状态话题
            # 尝试两种可能的话题格式
            try:
                # 先尝试没有中间斜杠的版本（用户环境实际情况）
                topic_name = f"/{self.robot_prefix}joint_states"
                self.subscriber = rospy.Subscriber(topic_name, JointState, self.joint_state_callback)
                # 测试是否能收到消息
                rospy.wait_for_message(topic_name, JointState, timeout=2)
            except rospy.ROSException:
                # 如果失败，尝试带中间斜杠的版本
                topic_name = f"/{self.robot_prefix}/joint_states"
                self.subscriber = rospy.Subscriber(topic_name, JointState, self.joint_state_callback)
        else:
            # 真实机器人使用Kinova驱动话题
            self.subscriber = rospy.Subscriber(f"/{self.robot_prefix}driver/out/joint_state", JointState, self.joint_state_callback)
    
    def joint_state_callback(self, data):
        """关节状态回调函数"""
        if len(data.position) >= self.nb_joints:
            for i in range(self.nb_joints):
                self.current_joint_positions[i] = np.rad2deg(data.position[i])
    
    def update_gui(self):
        """更新GUI显示"""
        # 更新关节状态显示
        for i, label in enumerate(self.joint_status_labels):
            label.setText(f"{self.current_joint_positions[i]:.2f}")
    
    def disconnect_robot(self):
        """断开与机器人的连接"""
        try:
            # 停止订阅者
            if hasattr(self, 'subscriber') and self.subscriber is not None:
                self.subscriber.unregister()
                self.subscriber = None
            QMessageBox.information(self, "断开连接", "已成功断开与机器人的连接!")
        except Exception as e:
            QMessageBox.warning(self, "断开连接失败", f"断开连接时出错: {str(e)}")
    
    def connect_robot(self):
        """连接机器人"""
        try:
            if self.simulation_radio.isChecked():
                # 仿真环境：检查关节状态话题
                # 尝试两种可能的话题格式
                topic_found = False
                error_msg = ""
                
                # 先尝试没有中间斜杠的版本（用户环境实际情况：/m1n6s200_joint_states）
                topic_name = f"/{self.robot_prefix}joint_states"
                try:
                    if rospy.wait_for_message(topic_name, JointState, timeout=2):
                        QMessageBox.information(self, "连接成功", f"已成功连接到{self.robot_prefix}仿真机器人!")
                        topic_found = True
                except rospy.ROSException as e:  # 与try保持相同缩进
                    error_msg += f"尝试话题 '{topic_name}' 失败: {str(e)}\n"  # 与except保持相同缩进
                
                # 如果第一种格式失败，尝试带中间斜杠的版本（标准格式：/m1n6s200_/joint_states）
                if not topic_found:
                    topic_name = f"/{self.robot_prefix}/joint_states"
                    try:
                        if rospy.wait_for_message(topic_name, JointState, timeout=2):
                            QMessageBox.information(self, "连接成功", f"已成功连接到{self.robot_prefix}仿真机器人!")
                            topic_found = True
                    except rospy.ROSException as e:  # 与try保持相同缩进
                        error_msg += f"尝试话题 '{topic_name}' 失败: {str(e)}\n"  # 与except保持相同缩进
                    
                if not topic_found:
                    QMessageBox.warning(self, "连接失败", f"无法连接到仿真机器人。\n\n尝试的话题：\n{error_msg}\n请检查ROS接口是否正确。")
                    return
            else:
                # 真实机器人：检查服务
                service_address = f"/{self.robot_prefix}driver/in/home_arm"
                rospy.wait_for_service(service_address, timeout=5)
                QMessageBox.information(self, "连接成功", f"已成功连接到{self.robot_prefix[:-1]}机器人!")
            
            # 启动订阅者
            self.start_ros_subscribers()
        except rospy.ROSException as e:
            QMessageBox.warning(self, "连接失败", f"无法连接到机器人。错误信息：\n{str(e)}\n请检查ROS接口是否正确。")


if __name__ == "__main__":
    # 初始化ROS节点
    if not rospy.core.is_initialized():
        rospy.init_node('kinova_gui', anonymous=True)
    
    # 创建Qt应用
    app = QApplication(sys.argv)
    
    # 设置样式
    app.setStyle("Fusion")
    
    # 创建并显示主窗口
    window = KinovaGUI()
    window.show()
    
    # 运行应用
    sys.exit(app.exec_())