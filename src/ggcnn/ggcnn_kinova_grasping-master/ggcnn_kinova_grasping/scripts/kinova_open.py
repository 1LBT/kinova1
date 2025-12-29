#!/usr/bin/env python

import rospy
import tf.transformations as tft
import numpy as np
import kinova_msgs.msg
import kinova_msgs.srv
import std_msgs.msg
import std_srvs.srv
import geometry_msgs.msg
from helpers.gripper_action_client import set_finger_positions
from helpers.position_action_client import position_client, move_to_position
from helpers.transforms import current_robot_pose, publish_tf_quaterion_as_transform, convert_pose, publish_pose_as_transform
from helpers.covariance import generate_cartesian_covariance

MOVING = False
CURR_Z = 0

class HandEyeCalibration:
    """手眼标定类 - 眼在手上配置"""
    def __init__(self):
        # 手眼标定矩阵 - 从相机坐标系到机械臂基坐标系
        # 根据您的实际标定结果修改这些参数
        self.cam_to_base_translation = np.array([-0.03496095263309274, -0.6940587002323341, 0.6572512763021039])
        self.cam_to_base_quaternion = np.array([-0.34481648755733546, -0.039982953815742794, 
                                               0.02919839600252768, 0.9373635404599806])
        
        # 计算旋转矩阵
        self.cam_to_base_rotation = tft.quaternion_matrix(self.cam_to_base_quaternion)[:3, :3]
        
    def transform_pose_cam_to_base(self, point_cam, angle_cam):
        """
        将抓取点从相机坐标系转换到机械臂基坐标系
        参数对应GGCNN输出的6个值: [x, y, z, ang, width, depth_center]
        """
        # 位置转换: p_base = R * p_cam + t
        point_base = np.dot(self.cam_to_base_rotation, point_cam) + self.cam_to_base_translation
        
        # 方向转换: 将抓取角度转换为四元数
        # GGCNN输出的角度是绕相机Z轴的旋转角度
        q_angle = tft.quaternion_from_euler(0, 0, angle_cam)  # 绕Z轴旋转
        
        # 相机到基座的方向转换
        q_base = tft.quaternion_multiply(self.cam_to_base_quaternion, q_angle)
        
        return point_base, q_base

def robot_wrench_callback(msg):
    """力矩回调函数，检测碰撞"""
    global MOVING
    if MOVING and msg.wrench.force.z < -2.0:
        MOVING = False
        rospy.logerr('检测到力，停止运动!')

def robot_position_callback(msg):
    """机械臂位置回调函数"""
    global CURR_Z
    CURR_Z = msg.pose.position.z

def move_to_pose(pose):
    """移动到指定位姿的封装函数"""
    p = pose.position
    o = pose.orientation
    move_to_position([p.x, p.y, p.z], [o.x, o.y, o.z, o.w])

def calculate_gripper_width(grasp_width_pixels, current_z):
    """
    计算夹爪宽度，与GGCNN节点中的计算保持一致
    对应GGCNN节点中的grip_width计算
    """
    # 与GGCNN节点相同的计算公式
    g_width = 2 * ((current_z + 0.07)) * np.tan(0.1 * grasp_width_pixels / 2.0 / 180.0 * np.pi) * 1000
    # 转换为电机位置 (4000-6800对应完全张开到闭合)
    g = min((1 - (min(g_width, 70)/70)) * (6800-4000) + 4000, 5500)
    return g

def execute_grasp():
    """执行抓取的主函数"""
    global MOVING, CURR_Z
    
    # 初始化手眼标定
    hand_eye = HandEyeCalibration()
    
    # 等待GGCNN发布的抓取命令
    rospy.loginfo("等待GGCNN抓取检测结果...")
    try:
        msg = rospy.wait_for_message('/ggcnn/out/command', std_msgs.msg.Float32MultiArray, timeout=10.0)
        d = list(msg.data)
        rospy.loginfo(f"收到抓取命令: {d}")
    except rospy.ROSException:
        rospy.logerr("等待GGCNN消息超时")
        return False
    
    # 检查数据长度是否符合预期 (应该是6个值)
    if len(d) != 6:
        rospy.logerr(f"抓取命令数据长度错误: 期望6, 实际{len(d)}")
        return False
    
    # 解析GGCNN输出 [x, y, z, ang, width, depth_center]
    x_cam, y_cam, z_cam = d[0], d[1], d[2]  # 相机坐标系中的位置
    grasp_angle = d[3]                       # 抓取角度 (弧度)
    grasp_width_pixels = d[4]                # 抓取宽度 (像素)
    depth_center = d[5]                      # 深度中心值 (用于碰撞检测)
    
    rospy.loginfo(f"抓取点-相机坐标系: 位置({x_cam:.3f}, {y_cam:.3f}, {z_cam:.3f}), 角度: {grasp_angle:.3f}rad")
    
    # 计算夹爪宽度
    gripper_position = calculate_gripper_width(grasp_width_pixels, CURR_Z)
    rospy.loginfo(f"设置夹爪位置: {gripper_position}")
    set_finger_positions([gripper_position, gripper_position])
    rospy.sleep(0.5)
    
    # 使用手眼标定将抓取点转换到基坐标系
    point_cam = np.array([x_cam, y_cam, z_cam])
    point_base, quat_base = hand_eye.transform_pose_cam_to_base(point_cam, grasp_angle)
    
    # 创建基坐标系中的抓取位姿
    gp_base = geometry_msgs.msg.Pose()
    gp_base.position.x = point_base[0]
    gp_base.position.y = point_base[1] 
    gp_base.position.z = point_base[2]
    gp_base.orientation.x = quat_base[0]
    gp_base.orientation.y = quat_base[1]
    gp_base.orientation.z = quat_base[2]
    gp_base.orientation.w = quat_base[3]
    
    # 发布TF用于可视化
    publish_pose_as_transform(gp_base, 'm1n6s200_link_base', 'Grasp_Point', 2.0)
    
    # 设置初始安全偏移
    initial_offset = 0.15  # 15cm的安全偏移
    gp_base.position.z += initial_offset
    
    # 精确运动阶段 - 禁用力控制提高精度
    rospy.loginfo("移动到抓取点上方的安全位置")
    stop_force_srv.call(kinova_msgs.srv.StopRequest())
    move_to_pose(gp_base)
    rospy.sleep(0.5)
    
    # 接近阶段 - 启用力控制避免碰撞
    rospy.loginfo("启用力控制，开始接近物体")
    start_force_srv.call(kinova_msgs.srv.StartRequest())
    rospy.sleep(0.25)
    
    # 重置到实际抓取高度
    gp_base.position.z -= initial_offset
    MOVING = True
    
    # 生成笛卡尔空间协方差用于速度控制
    cart_cov = generate_cartesian_covariance(0)
    
    # 速度控制下降 - 与GGCNN节点中的控制逻辑保持一致
    velo_pub = rospy.Publisher('/m1n6s200_driver/in/cartesian_velocity', kinova_msgs.msg.PoseVelocity, queue_size=1)
    rospy.loginfo("开始速度控制下降")
    
    descent_start_time = rospy.get_time()
    max_descent_time = 10.0  # 最大下降时间(秒)
    
    while MOVING and (rospy.get_time() - descent_start_time) < max_descent_time:
        if CURR_Z - 0.02 <= gp_base.position.z:
            rospy.loginfo("到达目标高度附近")
            break
            
        # 计算下降速度 (与GGCNN节点相同的控制逻辑)
        dz = gp_base.position.z - CURR_Z - 0.03  # 3cm的提前量
        MAX_VELO_Z = 0.08
        dz = max(min(dz, MAX_VELO_Z), -MAX_VELO_Z)
        
        # 发布速度命令
        v = np.array([0, 0, dz])
        vc = list(np.dot(v, cart_cov)) + [0, 0, 0]
        velo_pub.publish(kinova_msgs.msg.PoseVelocity(*vc))
        rospy.sleep(1/100.0)
    
    MOVING = False
    rospy.sleep(0.2)
    
    # 闭合夹爪执行抓取
    rospy.loginfo("闭合夹爪执行抓取")
    set_finger_positions([8000, 8000])  # 闭合位置
    rospy.sleep(0.5)
    
    # 抬升到安全高度
    rospy.loginfo("抬升到安全高度")
    gp_base.position.z += initial_offset
    move_to_pose(gp_base)
    
    # 禁用力控制
    stop_force_srv.call(kinova_msgs.srv.StopRequest())
    
    rospy.loginfo("抓取操作完成")
    return True

class GraspSystem:
    """抓取系统主类"""
    def __init__(self):
        self.home_positions = {
            'default_home': {
                'position': [0, -0.38, 0.35],  # 调整到更高位置确保安全
                'orientation': [0.99, 0, 0, np.sqrt(1-0.99**2)]
            },
            'scan_home': {
                'position': [0, -0.35, 0.40],
                'orientation': [0.95, 0.1, 0.1, 0.25]
            }
        }
        self.current_home = 'default_home'
    
    def move_to_home_position(self, home_name='default_home'):
        """移动到初始观测位置"""
        if home_name in self.home_positions:
            home_data = self.home_positions[home_name]
            success = move_to_position(home_data['position'], home_data['orientation'])
            if success:
                self.current_home = home_name
                rospy.loginfo(f"已移动到初始位置: {home_name}")
            return success
        return False

def main():
    rospy.init_node('ggcnn_grasp_executor')
    
    global start_force_srv, stop_force_srv
    
    # 初始化服务代理
    rospy.loginfo("等待力控制服务...")
    rospy.wait_for_service('/m1n6s200_driver/in/start_force_control')
    rospy.wait_for_service('/m1n6s200_driver/in/stop_force_control')
    
    start_force_srv = rospy.ServiceProxy('/m1n6s200_driver/in/start_force_control', kinova_msgs.srv.Start)
    stop_force_srv = rospy.ServiceProxy('/m1n6s200_driver/in/stop_force_control', kinova_msgs.srv.Stop)
    
    # 初始化状态监控
    wrench_sub = rospy.Subscriber('/m1n6s200_driver/out/tool_wrench', geometry_msgs.msg.WrenchStamped, 
                                robot_wrench_callback, queue_size=1)
    position_sub = rospy.Subscriber('/m1n6s200_driver/out/tool_pose', geometry_msgs.msg.PoseStamped, 
                                  robot_position_callback, queue_size=1)
    
    # 初始化抓取系统
    grasp_system = GraspSystem()
    
    rospy.loginfo("GGCNN抓取执行系统初始化完成")
    
    try:
        while not rospy.is_shutdown():
            # 打开夹爪准备抓取
            set_finger_positions([0, 0])
            rospy.sleep(0.5)
            
            # 移动到观测位置
            grasp_system.move_to_home_position('default_home')
            rospy.sleep(1.0)
            
            # 等待用户指令
            user_input = input("按回车开始GGCNN视觉抓取 (输入q退出): ")
            if user_input.lower() == 'q':
                break
                
            rospy.loginfo("开始执行GGCNN视觉抓取流程")
            
            # 执行抓取
            grasp_success = execute_grasp()
            
            if grasp_success:
                rospy.loginfo("抓取成功! 返回初始位置")
                # 返回初始位置
                grasp_system.move_to_home_position('default_home')
            else:
                rospy.logwarn("抓取失败，检查GGCNN节点和相机数据")
            
            rospy.sleep(1.0)
            
    except KeyboardInterrupt:
        rospy.loginfo("程序被用户中断")
    except Exception as e:
        rospy.logerr(f"程序执行错误: {e}")
    finally:
        # 确保机械臂回到安全位置
        grasp_system.move_to_home_position('default_home')
        rospy.loginfo("程序结束")

if __name__ == '__main__':
    main()