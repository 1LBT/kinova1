#! /usr/bin/env python

import time
import numpy as np
import torch
import torch.nn.functional as F
from torchvision import transforms
import sys
import cv2
import scipy.ndimage 
from skimage.draw import disk
from skimage.feature import peak_local_max

import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray

import torch.nn as nn
from collections import OrderedDict
import os

bridge = CvBridge()
sys.path.append('/home/liu/kinova_ws/src/ggcnn/ggcnn-master/models')
try:
    from ggcnn2 import GGCNN2
    print("成功从外部文件导入 GGCNN2 模型定义")
except ImportError as e:
    print(f"无法导入 GGCNN2 模型定义: {e}")
def load_ggcnn2(model_path):
    """加载 GGCNN2 模型
    参数:
        model_path (str): 模型文件路径
    返回:
        model (nn.Module): 加载的模型
        device (torch.device): 使用的设备
    """
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"使用设备: {device}")
    # 检查文件是否存在
    if not os.path.exists(model_path):
        print(f"错误: 文件不存在: {model_path}")
        # 尝试可能的路径变体
        possible_paths = [
            '/' + model_path,  # 添加根目录
            os.path.expanduser('~') + '/' + model_path.lstrip('home/'),  # 扩展到用户目录
            os.path.join(os.path.dirname(__file__), model_path),  # 相对于脚本目录
        ]
        
        for path in possible_paths:
            if os.path.exists(path):
                model_path = path
                print(f"找到文件: {path}")
                break
        else:
            raise FileNotFoundError(f"无法找到模型文件: {model_path}")
    # 加载模型文件
    try:
        # 尝试使用 weights_only=True（更安全）
        state_dict = torch.load(model_path, map_location=device, weights_only=True)
        print("使用 weights_only=True 成功加载状态字典")
    except:
        print("使用 weights_only=True 失败，尝试 weights_only=False")
        state_dict = torch.load(model_path, map_location=device, weights_only=False)
    
    # 检查状态字典键名
    print("状态字典键名示例:")
    for i, key in enumerate(list(state_dict.keys())):
        if i < 5:  # 只显示前5个键
            print(f"  {key}: {state_dict[key].shape}")
    
    # 创建模型实例
    model = GGCNN2()
    
    # 加载状态字典到模型
    try:
        model.load_state_dict(state_dict)
        print("成功加载状态字典到模型")
    except Exception as e:
        print(f"加载状态字典失败: {e}")
        print("尝试检查键名不匹配...")
        
        # 检查键名不匹配
        model_keys = set(model.state_dict().keys())
        state_keys = set(state_dict.keys())
        
        print(f"模型键名: {len(model_keys)}")
        print(f"状态字典键名: {len(state_keys)}")
        
        missing_keys = model_keys - state_keys
        unexpected_keys = state_keys - model_keys
        
        if missing_keys:
            print(f"缺失的键: {missing_keys}")
        if unexpected_keys:
            print(f"意外的键: {unexpected_keys}")
        
        raise
    
    model.to(device)
    model.eval()
    
    return model, device
model_path = '/home/liu/kinova_ws/src/ggcnn/ggcnn2_weights_cornell/epoch_50_cornell_statedict.pt'
model, device = load_ggcnn2(model_path)
# # Load the Network - PyTorch version
# MODEL_FILE = '/home/liu/kinova_ws/src/ggcnn/ggcnn2_weights_cornell/epoch_50_cornell_statedict.pt'  # 改为PyTorch模型文件
# device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

# # 加载PyTorch模型
# model = torch.load(MODEL_FILE, map_location=device,weights_only=False)
# model.eval()  # 设置为评估模式

rospy.init_node('ggcnn_detection')

# Output publishers.
grasp_pub = rospy.Publisher('ggcnn/img/grasp', Image, queue_size=1)
grasp_plain_pub = rospy.Publisher('ggcnn/img/grasp_plain', Image, queue_size=1)
depth_pub = rospy.Publisher('ggcnn/img/depth', Image, queue_size=1)
ang_pub = rospy.Publisher('ggcnn/img/ang', Image, queue_size=1)
cmd_pub = rospy.Publisher('ggcnn/out/command', Float32MultiArray, queue_size=1)

# Initialise some globals.
prev_mp = np.array([150, 150])
ROBOT_Z = 0

# 不再需要TensorFlow图
# graph = tf.get_default_graph()

# Get the camera parameters
camera_info_msg = rospy.wait_for_message('/camera/depth/camera_info', CameraInfo)
K = camera_info_msg.K
fx = K[0]
cx = K[2]
fy = K[4]
cy = K[5]

# 定义预处理转换
preprocess = transforms.Compose([
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.5], std=[0.5])  # 将值范围从[0,1]转换为[-1,1]
])

# Execution Timing
class TimeIt:
    def __init__(self, s):
        self.s = s
        self.t0 = None
        self.t1 = None
        self.print_output = False

    def __enter__(self):
        self.t0 = time.time()

    def __exit__(self, t, value, traceback):
        self.t1 = time.time()
        if self.print_output:
            print('%s: %s' % (self.s, self.t1 - self.t0))

def robot_pos_callback(data):
    global ROBOT_Z
    ROBOT_Z = data.pose.position.z

def depth_callback(depth_message):
    global model
    global prev_mp
    global ROBOT_Z
    global fx, cx, fy, cy

    with TimeIt('Crop'):
        depth = bridge.imgmsg_to_cv2(depth_message)
        depth = depth.astype(np.float32) / 1000.0
        # Crop a square out of the middle of the depth and resize it to 300 * 300
        crop_size = 400
        depth_crop = cv2.resize(depth[(480-crop_size)//2:(480-crop_size)//2+crop_size, 
                                     (640-crop_size)//2:(640-crop_size)//2+crop_size], 
                               (300, 300))

        # Replace nan with 0 for inpainting.
        depth_crop = depth_crop.copy()
        depth_nan = np.isnan(depth_crop).copy()
        depth_crop[depth_nan] = 0

    with TimeIt('Inpaint'):
        # open cv inpainting does weird things at the border.
        depth_crop = cv2.copyMakeBorder(depth_crop, 1, 1, 1, 1, cv2.BORDER_DEFAULT)

        mask = (depth_crop == 0).astype(np.uint8)
        # Scale to keep as float, but has to be in bounds -1:1 to keep opencv happy.
        depth_scale = np.abs(depth_crop).max()
        depth_crop = depth_crop.astype(np.float32)/depth_scale  # Has to be float32, 64 not supported.

        depth_crop = cv2.inpaint(depth_crop, mask, 1, cv2.INPAINT_NS)

        # Back to original size and value range.
        depth_crop = depth_crop[1:-1, 1:-1]
        depth_crop = depth_crop * depth_scale

    with TimeIt('Calculate Depth'):
        # Figure out roughly the depth in mm of the part between the grippers for collision avoidance.
        depth_center = depth_crop[100:141, 130:171].flatten()
        depth_center.sort()
        depth_center = depth_center[:10].mean() * 1000.0

    with TimeIt('Inference'):
        # PyTorch预处理和推理
        # 归一化并转换为PyTorch张量
        depth_normalized = np.clip((depth_crop - depth_crop.mean()), -1, 1)
        
        # 转换为PyTorch张量并添加批次和通道维度
        input_tensor = torch.from_numpy(depth_normalized).unsqueeze(0).unsqueeze(0).float()
        input_tensor = input_tensor.to(device)
        
        # 模型推理
        with torch.no_grad():
            pred_out = model(input_tensor)
        
        # 转换为NumPy数组
        if isinstance(pred_out, (list, tuple)):
            # 如果模型返回多个输出
            pred_out = [p.cpu().numpy() for p in pred_out]
        else:
            # 如果模型返回单个张量
            pred_out = pred_out.cpu().numpy()
        
        # 根据模型输出调整形状
        # 假设输出顺序为 [points, cos, sin, width]
        points_out = pred_out[0].squeeze()
        cos_out = pred_out[1].squeeze()
        sin_out = pred_out[2].squeeze()
        width_out = pred_out[3].squeeze() * 150.0  # Scaled 0-150:0-1
        
        points_out[depth_nan] = 0

    with TimeIt('Trig'):
        # Calculate the angle map.
        ang_out = np.arctan2(sin_out, cos_out)/2.0

    with TimeIt('Filter'):
        # Filter the outputs.
        points_out = scipy.ndimage.gaussian_filter(points_out, 5.0)  # 3.0
        ang_out = scipy.ndimage.gaussian_filter(ang_out, 2.0)

    with TimeIt('Control'):
        # Calculate the best pose from the camera intrinsics.
        maxes = None

        ALWAYS_MAX = False  # Use ALWAYS_MAX = True for the open-loop solution.

        if ROBOT_Z > 0.34 or ALWAYS_MAX:  # > 0.34 initialises the max tracking when the robot is reset.
            # Track the global max.
            max_pixel = np.array(np.unravel_index(np.argmax(points_out), points_out.shape))
            prev_mp = max_pixel.astype(int)
        else:
            # Calculate a set of local maxes.  Choose the one that is closes to the previous one.
            maxes = peak_local_max(points_out, min_distance=10, threshold_abs=0.1, num_peaks=3)
            if maxes.shape[0] == 0:
                return
            max_pixel = maxes[np.argmin(np.linalg.norm(maxes - prev_mp, axis=1))]

            # Keep a global copy for next iteration.
            prev_mp = (max_pixel * 0.25 + prev_mp * 0.75).astype(int)

        ang = ang_out[max_pixel[0], max_pixel[1]]
        width = width_out[max_pixel[0], max_pixel[1]]

        # Convert max_pixel back to uncropped/resized image coordinates in order to do the camera transform.
        max_pixel = ((np.array(max_pixel) / 300.0 * crop_size) + np.array([(480 - crop_size)//2, (640 - crop_size) // 2]))
        max_pixel = np.round(max_pixel).astype(int)

        point_depth = depth[max_pixel[0], max_pixel[1]]

        # These magic numbers are my camera intrinsic parameters.
        x = (max_pixel[1] - cx)/(fx) * point_depth
        y = (max_pixel[0] - cy)/(fy) * point_depth
        z = point_depth

        if np.isnan(z):
            return

    with TimeIt('Draw'):
        # Draw grasp markers on the points_out and publish it. (for visualisation)
        grasp_img = np.zeros((300, 300, 3), dtype=np.uint8)
        grasp_img[:,:,2] = (points_out * 255.0)

        grasp_img_plain = grasp_img.copy()

        rr, cc = disk((prev_mp[0], prev_mp[1]), 5)
        grasp_img[rr, cc, 0] = 0
        grasp_img[rr, cc, 1] = 255
        grasp_img[rr, cc, 2] = 0

    with TimeIt('Publish'):
        # Publish the output images (not used for control, only visualisation)
        grasp_img = bridge.cv2_to_imgmsg(grasp_img, 'bgr8')
        grasp_img.header = depth_message.header
        grasp_pub.publish(grasp_img)

        grasp_img_plain = bridge.cv2_to_imgmsg(grasp_img_plain, 'bgr8')
        grasp_img_plain.header = depth_message.header
        grasp_plain_pub.publish(grasp_img_plain)

        depth_pub.publish(bridge.cv2_to_imgmsg(depth_crop))

        ang_pub.publish(bridge.cv2_to_imgmsg(ang_out))

        # Output the best grasp pose relative to camera.
        cmd_msg = Float32MultiArray()
        cmd_msg.data = [x, y, z, ang, width, depth_center]
        cmd_pub.publish(cmd_msg)

depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, depth_callback, queue_size=1)
robot_pos_sub = rospy.Subscriber('/m1n6s200_driver/out/tool_pose', PoseStamped, robot_pos_callback, queue_size=1)

while not rospy.is_shutdown():
    rospy.spin()