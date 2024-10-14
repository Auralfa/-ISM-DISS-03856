#!/usr/bin/env python

import rospy
import pymongo
from sensor_msgs.msg import PointCloud, PointCloud2, Imu, Image
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from datetime import datetime
import os
import cv2
import numpy as np

# 连接到MongoDB
client = pymongo.MongoClient("mongodb://localhost:27017/")
db = client['sensor_fusion']  # 创建一个数据库
collection_sensors = db['sensor_data']  # 创建一个集合存储传感器数据
collection_odom = db['odom_data']  # 创建一个集合存储位姿估计结果

# 数据插入到MongoDB中的函数
def insert_data(collection, data_list):
    try:
        if data_list:
            collection.insert_many(data_list)
            rospy.loginfo("Data inserted into MongoDB successfully")
    except Exception as e:
        rospy.logerr(f"Failed to insert data: {e}")

# 查询数据
def query_data(collection, query):
    try:
        results = collection.find(query)
        return list(results)
    except Exception as e:
        rospy.logerr(f"Failed to query data: {e}")
        return []

# 清理过时数据
def cleanup_old_data(collection, cutoff_time):
    try:
        result = collection.delete_many({'timestamp': {'$lt': cutoff_time}})
        rospy.loginfo(f"Deleted {result.deleted_count} old records from {collection.name}")
    except Exception as e:
        rospy.logerr(f"Failed to clean up data: {e}")

# 保存图像为文件，并返回文件路径
def save_image(image_data, folder_path):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # 获取当前时间作为文件名
    file_path = os.path.join(folder_path, f"{timestamp}.png")
    cv2.imwrite(file_path, image_data)
    return file_path

# 处理雷达点云数据（PointCloud类型）
def radar_pcl_callback(msg):
    pcl_data = {
        'timestamp': msg.header.stamp.to_nsec(),
        'frame_id': msg.header.frame_id,
        'points': [{'x': p.x, 'y': p.y, 'z': p.z} for p in msg.points]
    }
    insert_data(collection_sensors, [{'type': 'radar_enhanced_pcl', 'data': pcl_data}])

# 处理Hesai激光雷达数据（PointCloud2类型）
def hesai_pcl_callback(msg):
    pcl_data = {
        'timestamp': msg.header.stamp.to_to_nsec(),
        'frame_id': msg.header.frame_id,
        'points': []
    }
    # 解析PointCloud2格式
    for point in point_cloud2.read_points(msg, skip_nans=True):
        pcl_data['points'].append({'x': point[0], 'y': point[1], 'z': point[2]})
    
    insert_data(collection_sensors, [{'type': 'hesai_pandar', 'data': pcl_data}])

# 处理IMU数据
def imu_callback(msg):
    imu_data = {
        'timestamp': msg.header.stamp.to_nsec(),
        'orientation': {
            'x': msg.orientation.x,
            'y': msg.orientation.y,
            'z': msg.orientation.z,
            'w': msg.orientation.w
        },
        'angular_velocity': {
            'x': msg.angular_velocity.x,
            'y': msg.angular_velocity.y,
            'z': msg.angular_velocity.z
        },
        'linear_acceleration': {
            'x': msg.linear_acceleration.x,
            'y': msg.linear_acceleration.y,
            'z': msg.linear_acceleration.z
        }
    }
    insert_data(collection_sensors, [{'type': 'imu', 'data': imu_data}])

# 处理左相机图像数据
def left_image_callback(msg):
    image_data = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)  # 将图像数据转换为numpy数组
    image_path = save_image(image_data, '/home/zw/scout_ws/src/fus_pkg/data/sensor/stereo')  # 设定图像保存路径
    image_info = {
        'timestamp': msg.header.stamp.to_nsec(),
        'frame_id': msg.header.frame_id,
        'file_path': image_path,
        'encoding': msg.encoding,
        'height': msg.height,
        'width': msg.width,
        'step': msg.step
    }
    insert_data(collection_sensors, [{'type': 'left_image', 'data': image_info}])

# 处理右相机图像数据
def right_image_callback(msg):
    image_data = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)  # 将图像数据转换为numpy数组
    image_path = save_image(image_data, '/home/zw/scout_ws/src/fus_pkg/data/sensor/stereo')  # 设定图像保存路径
    image_info = {
        'timestamp': msg.header.stamp.to_nsec(),
        'frame_id': msg.header.frame_id,
        'file_path': image_path,
        'encoding': msg.encoding,
        'height': msg.height,
        'width': msg.width,
        'step': msg.step
    }
    insert_data(collection_sensors, [{'type': 'right_image', 'data': image_info}])

# 处理ORB-SLAM3位姿估计结果
def orb_slam3_odom_callback(msg):
    odom_data = {
        'timestamp': msg.header.stamp.to_nsec(),
        'frame_id': msg.header.frame_id,
        'position': {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        },
        'orientation': {
            'x': msg.pose.pose.orientation.x,
            'y': msg.pose.pose.orientation.y,
            'z': msg.pose.pose.orientation.z,
            'w': msg.pose.pose.orientation.w
        }
    }
    insert_data(collection_odom, [{'type': 'orb_slam3_odom', 'data': odom_data}])

# 处理A-LOAM位姿估计结果
def aloam_odom_callback(msg):
    odom_data = {
        'timestamp': msg.header.stamp.to_nsec(),
        'frame_id': msg.header.frame_id,
        'position': {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        },
        'orientation': {
            'x': msg.pose.pose.orientation.x,
            'y': msg.pose.pose.orientation.y,
            'z': msg.pose.pose.orientation.z,
            'w': msg.pose.pose.orientation.w
        }
    }
    insert_data(collection_odom, [{'type': 'aloam_odom', 'data': odom_data}])

# 处理4D-Radar-SLAM位姿估计结果
def radar_slam_odom_callback(msg):
    odom_data = {
        'timestamp': msg.header.stamp.to_nsec(),
        'frame_id': msg.header.frame_id,
        'position': {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        },
        'orientation': {
            'x': msg.pose.pose.orientation.x,
            'y': msg.pose.pose.orientation.y,
            'z': msg.pose.pose.orientation.z,
            'w': msg.pose.pose.orientation.w
        }
    }
    insert_data(collection_odom, [{'type': 'radar_slam_odom', 'data': odom_data}])

# ROS节点初始化与订阅
if __name__ == '__main__':
    rospy.init_node('sensor_data_processor')

    rospy.Subscriber("/radar_enhanced_pcl", PointCloud, radar_pcl_callback)
    rospy.Subscriber("/hesai_pandar", PointCloud2, hesai_pcl_callback)
    rospy.Subscriber("/stereo_inertial_publisher/imu", Imu, imu_callback)
    rospy.Subscriber("/stereo_inertial_publisher/left/image_rect", Image, left_image_callback)
    rospy.Subscriber("/stereo_inertial_publisher/right/image_rect", Image, right_image_callback)
    
    rospy.Subscriber("/orb3_odom", Odometry, orb_slam3_odom_callback)
    rospy.Subscriber("/aft_mapped_to_init", Odometry, aloam_odom_callback)
    rospy.Subscriber("/radar_graph_slam/aftmapped_to_init", Odometry, radar_slam_odom_callback)

    # 清理过时数据，设定保留的时间阈值（保留过去 1 小时的数据）
    cleanup_cutoff_time = rospy.Time.now().to_nsec() - 3600 * 1e9  # 1 小时的纳秒数
    cleanup_old_data(collection_sensors, cleanup_cutoff_time)
    cleanup_old_data(collection_odom, cleanup_cutoff_time)

    rospy.spin()
