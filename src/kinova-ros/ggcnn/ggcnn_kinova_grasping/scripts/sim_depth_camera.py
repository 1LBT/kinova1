#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import tf2_geometry_msgs
import tf.transformations as tf_trans

class SimDepthCamera:
    def __init__(self):
        rospy.init_node('sim_depth_camera')
        
        self.bridge = CvBridge()
        
        # 发布深度图像和相机信息
        self.depth_pub = rospy.Publisher('/camera/depth/image_rect_raw', Image, queue_size=1)
        self.camera_info_pub = rospy.Publisher('/camera/depth/camera_info', CameraInfo, queue_size=1)
        
        # 相机内参 (模拟Realsense SR300)
        self.camera_info = CameraInfo()
        self.camera_info.width = 640
        self.camera_info.height = 480
        self.camera_info.K = [525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0]
        self.camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.camera_info.P = [525.0, 0.0, 319.5, 0.0, 0.0, 525.0, 239.5, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        # 模拟物体位置 - 调整为在相机视野中心
        self.objects = [
            {'name': 'grasp_cube', 'pose': [0.3, 0.0, 0.3], 'size': [0.05, 0.05, 0.05]},  # 移到相机前方
            {'name': 'grasp_cylinder', 'pose': [0.35, 0.1, 0.3], 'size': [0.03, 0.08]},  # 移到相机前方
            {'name': 'grasp_sphere', 'pose': [0.25, -0.1, 0.3], 'size': [0.04]}  # 移到相机前方
        ]
        
        self.rate = rospy.Rate(30)  # 30Hz
        
        # 定义相机相对于机器人基坐标系的变换（位置和朝向）
        self.camera_position = np.array([0.3, 0.0, 0.5])  # 相机位置
        # 相机朝向的四元数（直接朝向下）
        self.camera_quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        
        # 创建静态TF变换发布器
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        # 定义相机坐标系相对于机器人基坐标系的变换
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "m1n6s200_link_base"
        static_transformStamped.child_frame_id = "camera_depth_optical_frame"
        
        # 设置相机位置
        static_transformStamped.transform.translation.x = self.camera_position[0]
        static_transformStamped.transform.translation.y = self.camera_position[1]
        static_transformStamped.transform.translation.z = self.camera_position[2]
        
        # 设置相机朝向
        static_transformStamped.transform.rotation.x = self.camera_quaternion[0]
        static_transformStamped.transform.rotation.y = self.camera_quaternion[1]
        static_transformStamped.transform.rotation.z = self.camera_quaternion[2]
        static_transformStamped.transform.rotation.w = self.camera_quaternion[3]
        
        # 发布静态TF变换
        self.static_broadcaster.sendTransform(static_transformStamped)
        rospy.loginfo("已发布相机与机器人基坐标系之间的静态TF变换")
        
        # 预计算相机变换矩阵
        # 将四元数转换为旋转矩阵
        self.rotation_matrix = tf_trans.quaternion_matrix(self.camera_quaternion)
        # 设置平移向量
        self.rotation_matrix[0:3, 3] = self.camera_position
        
    def generate_depth_image(self):
        """生成模拟的深度图像"""
        # 创建深度图像，背景为500毫米
        depth_image = np.ones((480, 640), dtype=np.float32) * 500.0
        
        try:
            # 直接在图像中心附近绘制物体，确保在裁剪区域内
            # 中心物体（立方体）
            cv2.circle(depth_image, (320, 240), 30, 400, -1)  # 深度400mm
            
            # 右侧物体（圆柱体）
            cv2.circle(depth_image, (380, 240), 25, 350, -1)  # 深度350mm
            
            # 左侧物体（球体）
            cv2.circle(depth_image, (260, 240), 20, 450, -1)  # 深度450mm
            
            # 在裁剪区域（中间400x400）内添加更多物体
            # 上方物体
            cv2.circle(depth_image, (320, 200), 15, 380, -1)  # 深度380mm
            
            # 下方物体
            cv2.circle(depth_image, (320, 280), 15, 420, -1)  # 深度420mm
            
        except Exception as e:
            rospy.logwarn(f"生成深度图像时出错: {e}")
            
        return depth_image
    
    def run(self):
        while not rospy.is_shutdown():
            # 生成深度图像
            depth_image = self.generate_depth_image()
            
            # 发布深度图像
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image.astype(np.uint16), '16UC1')
            depth_msg.header.stamp = rospy.Time.now()
            depth_msg.header.frame_id = 'camera_depth_optical_frame'
            self.depth_pub.publish(depth_msg)
            
            # 发布相机信息
            self.camera_info.header.stamp = rospy.Time.now()
            self.camera_info.header.frame_id = 'camera_depth_optical_frame'
            self.camera_info_pub.publish(self.camera_info)
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        camera = SimDepthCamera()
        camera.run()
    except rospy.ROSInterruptException:
        pass

    with TimeIt('Inpaint'):
        # open cv inpainting does weird things at the border.
        depth_crop = cv2.copyMakeBorder(depth_crop, 1, 1, 1, 1, cv2.BORDER_DEFAULT)

        mask = (depth_crop == 0).astype(np.uint8)
        # Scale to keep as float, but has to be in bounds -1:1 to keep opencv happy.
        depth_scale = np.abs(depth_crop).max()
        
        # 添加安全检查，避免除零错误
        if depth_scale == 0:
            depth_scale = 1.0  # 设置默认值1.0避免除零
        
        depth_crop = depth_crop.astype(np.float32)/depth_scale  # Has to be float32, 64 not supported.

        depth_crop = cv2.inpaint(depth_crop, mask, 1, cv2.INPAINT_NS)

        # Back to original size and value range.
        depth_crop = depth_crop[1:-1, 1:-1]
        depth_crop = depth_crop * depth_scale