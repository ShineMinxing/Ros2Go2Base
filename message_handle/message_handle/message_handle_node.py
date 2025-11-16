#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Pose
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros

def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    return qx, qy, qz, qw

def euler_from_matrix(matrix):
    epsilon = 1e-6
    R = np.array(matrix, dtype=np.float64)
    if abs(R[2,0]) < 1.0 - epsilon:
        pitch = math.asin(-R[2,0])
        roll  = math.atan2(R[2,1], R[2,2])
        yaw   = math.atan2(R[1,0], R[0,0])
    else:
        sign = math.copysign(1.0, R[2,0])
        pitch = -sign * math.pi/2
        roll  = -sign * math.atan2(R[0,1], R[0,2])
        yaw   = 0.0
    return roll, pitch, yaw

def euler_to_matrix(roll, pitch, yaw):
    Rx = np.array([[1,0,0],[0,math.cos(roll),-math.sin(roll)],[0,math.sin(roll),math.cos(roll)]])
    Ry = np.array([[math.cos(pitch),0,math.sin(pitch)],[0,1,0],[-math.sin(pitch),0,math.cos(pitch)]])
    Rz = np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]])
    return Rz @ Ry @ Rx

def pose_to_matrix(p: Pose) -> np.ndarray:
    x, y, z = p.position.x, p.position.y, p.position.z
    qx, qy, qz, qw = p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
    R = np.array([
        [1-2*qy*qy-2*qz*qz,   2*qx*qy-2*qz*qw,     2*qx*qz+2*qy*qw],
        [2*qx*qy+2*qz*qw,     1-2*qx*qx-2*qz*qz,   2*qy*qz-2*qx*qw],
        [2*qx*qz-2*qy*qw,     2*qy*qz+2*qx*qw,     1-2*qx*qx-2*qy*qy]
    ], dtype=np.float64)
    T = np.eye(4); T[:3,:3] = R; T[:3,3] = [x, y, z]
    return T

def matrix_to_pose(T: np.ndarray) -> Pose:
    R = T[:3,:3]
    t = T[:3,3]
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2,1] - R[1,2]) / S
        qy = (R[0,2] - R[2,0]) / S
        qz = (R[1,0] - R[0,1]) / S
    else:
        i = np.argmax([R[0,0], R[1,1], R[2,2]])
        if i == 0:
            S = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
            qx = 0.25 * S
            qy = (R[0,1] + R[1,0]) / S
            qz = (R[0,2] + R[2,0]) / S
            qw = (R[2,1] - R[1,2]) / S
        elif i == 1:
            S = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
            qx = (R[0,1] + R[1,0]) / S
            qy = 0.25 * S
            qz = (R[1,2] + R[2,1]) / S
            qw = (R[0,2] - R[2,0]) / S
        else:
            S = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
            qx = (R[0,2] + R[2,0]) / S
            qy = (R[1,2] + R[2,1]) / S
            qz = 0.25 * S
            qw = (R[1,0] - R[0,1]) / S
    out = Pose()
    out.position.x, out.position.y, out.position.z = t.tolist()
    out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = qx, qy, qz, qw
    return out

class MessageHandleNode(Node):
    def __init__(self, *, cli_args=None):
        super().__init__('message_handle_node', cli_args=cli_args)

        # —— 从参数服务器读取所有配置 —— #
        p = self.declare_parameter
        self.sub_pc_topic   = p('sub_pointcloud_topic', '/NoYamlRead/Go2Lidar').value
        self.pub_scan_topic = p('pub_laserscan_topic',  '/NoYamlRead/Scan').value
        self.sub_odom_topic   = p('sub_odom_topic',   '/NoYamlRead/Odom').value
        self.sub_odom2d_topic = p('sub_odom2d_topic', '/NoYamlRead/Odom_2D').value

        self.angle_min       = p('angle_min',       -math.pi/2).value
        self.angle_max       = p('angle_max',        math.pi/2).value
        self.angle_increment = p('angle_increment',  0.0056).value
        self.range_min       = p('range_min',        0.1).value
        self.range_max       = p('range_max',       30.0).value
        self.min_height      = p('min_height',       0.15).value
        self.max_height      = p('max_height',        3.0).value

        self.map_frame       = p('map_frame',      'map').value
        self.odom_frame      = p('odom_frame',     'odom').value
        self.base_frame      = p('base_frame',     'base_link').value
        self.base_frame_2d   = p('base_frame_2d',  'base_link_2D').value

        self.utlidar_frame   = p('utlidar_frame',  'utlidar_lidar').value
        self.static_X_Lidar  = p('static_X_Lidar', 0.28945).value
        self.static_Y_Lidar  = p('static_Y_Lidar', 0.0).value
        self.static_Z_Lidar  = p('static_Z_Lidar', -0.046825).value
        self.static_r_Lidar  = p('static_r_Lidar',  0.0).value
        self.static_p_Lidar  = p('static_p_Lidar',  2.8782).value
        self.static_y_Lidar  = p('static_y_Lidar',  0.0).value

        self.gimbal_frame    = p('gimbal_frame',  'gimbal').value
        self.pub_gimbal_odom_topic = p('pub_gimbal_odom_topic', '/SMX/Odom_gimbal').value
        self.static_X_Gimbal = p('static_X_Gimbal', 0.28945).value  # 注意：大写 X
        self.static_Y_Gimbal = p('static_Y_Gimbal', 0.0).value
        self.static_Z_Gimbal = p('static_Z_Gimbal', 0.18).value
        self.static_r_Gimbal = p('static_r_Gimbal',  0.0).value
        self.static_p_Gimbal = p('static_p_Gimbal',  0.0).value
        self.static_y_Gimbal = p('static_y_Gimbal',  0.0).value

        self.imu_frame       = p('imu_frame',  'base_imu').value
        self.static_X_Imu    = p('static_X_Imu', -0.02557).value
        self.static_Y_Imu    = p('static_Y_Imu', 0.0).value
        self.static_Z_Imu    = p('static_Z_Imu', 0.04232).value
        self.static_r_Imu    = p('static_r_Imu',  0.0).value
        self.static_p_Imu    = p('static_p_Imu',  0.0).value
        self.static_y_Imu    = p('static_y_Imu',  0.0).value

        # —— TF、订阅、发布 —— #
        self.tf_buffer    = tf2_ros.Buffer()
        self.tf_listener  = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster1 = tf2_ros.TransformBroadcaster(self)
        self.tf_broadcaster2 = tf2_ros.TransformBroadcaster(self)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.sub_cloud = self.create_subscription(
            PointCloud2, self.sub_pc_topic, self.pointcloud_callback, 10)
        self.pub_scan  = self.create_publisher(LaserScan, self.pub_scan_topic, 10)
        self.pub_gimbal_odom = self.create_publisher(Odometry, self.pub_gimbal_odom_topic, 10)

        self.odom_counter   = 0
        self.odom2d_counter = 0
        self.sub_odom   = self.create_subscription(
            Odometry, self.sub_odom_topic,   self.odom_callback,   10)
        self.sub_odom2d = self.create_subscription(
            Odometry, self.sub_odom2d_topic, self.odom2d_callback, 10)

        # 发布静态 TF（用定时器重复广播，避免 RViz 启动时错过）
        self.create_timer(0.2, self.publish_base_to_utlidar_transform, callback_group=None)
        self.create_timer(0.2, self.publish_base_to_gimbal_transform, callback_group=None)
        self.create_timer(0.2, self.publish_base_to_imu_transform, callback_group=None) 
        self.create_timer(1.0, self.publish_map_to_odom_transform, callback_group=None)

        # 预计算 base->gimbal 4x4 外参矩阵（缓存）
        qx, qy, qz, qw = euler_to_quaternion(self.static_r_Gimbal, self.static_p_Gimbal, self.static_y_Gimbal)
        R_bg = np.array([
            [1-2*qy*qy-2*qz*qz,   2*qx*qy-2*qz*qw,     2*qx*qz+2*qy*qw],
            [2*qx*qy+2*qz*qw,     1-2*qx*qx-2*qz*qz,   2*qy*qz-2*qx*qw],
            [2*qx*qz-2*qy*qw,     2*qy*qz+2*qx*qw,     1-2*qx*qx-2*qy*qy]
        ], dtype=np.float64)
        self.T_base_gimbal = np.eye(4)
        self.T_base_gimbal[:3,:3] = R_bg
        self.T_base_gimbal[:3, 3] = np.array([self.static_X_Gimbal, self.static_Y_Gimbal, self.static_Z_Gimbal], dtype=np.float64)

        self.get_logger().info("MessageHandleNode 已启动")

    def publish_base_to_utlidar_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id  = self.base_frame
        t.child_frame_id   = self.utlidar_frame
        t.transform.translation.x = self.static_X_Lidar
        t.transform.translation.y = self.static_Y_Lidar
        t.transform.translation.z = self.static_Z_Lidar
        qx, qy, qz, qw = euler_to_quaternion(self.static_r_Lidar, self.static_p_Lidar, self.static_y_Lidar)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.static_broadcaster.sendTransform([t])

    def publish_base_to_gimbal_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id  = self.base_frame
        t.child_frame_id   = self.gimbal_frame
        t.transform.translation.x = self.static_X_Gimbal
        t.transform.translation.y = self.static_Y_Gimbal
        t.transform.translation.z = self.static_Z_Gimbal
        qx, qy, qz, qw = euler_to_quaternion(self.static_r_Gimbal, self.static_p_Gimbal, self.static_y_Gimbal)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.static_broadcaster.sendTransform([t])
    
    def publish_base_to_imu_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id  = self.base_frame
        t.child_frame_id   = self.imu_frame
        t.transform.translation.x = self.static_X_Imu
        t.transform.translation.y = self.static_Y_Imu
        t.transform.translation.z = self.static_Z_Imu
        qx, qy, qz, qw = euler_to_quaternion(
            self.static_r_Imu, self.static_p_Imu, self.static_y_Imu
        )
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.static_broadcaster.sendTransform([t])

    def publish_map_to_odom_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id  = self.map_frame
        t.child_frame_id   = self.odom_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, 0.0)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.static_broadcaster.sendTransform([t])

    def transform_to_matrix(self, transform: TransformStamped) -> np.ndarray:
        t = transform.transform.translation
        r = transform.transform.rotation
        x,y,z,w = r.x, r.y, r.z, r.w
        R = np.array([
            [1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w],
            [2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w],
            [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y]
        ])
        T = np.eye(4); T[:3,:3]=R; T[:3,3]=[t.x,t.y,t.z]
        return T

    def pointcloud_callback(self, msg: PointCloud2):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.base_frame_2d,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().warn(f'TF lookup 错误: {e}')
            return

        T = self.transform_to_matrix(trans)
        pts_list = [ [p[0], p[1], p[2]] for p in pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True) ]
        if not pts_list:
            return
        pts = np.array(pts_list, dtype=np.float64)
        ones = np.ones((pts.shape[0],1))
        hom = np.hstack((pts,ones))
        pts_b = (T @ hom.T).T[:,:3]

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_b = pc2.create_cloud(msg.header, fields, pts_b.tolist())

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.base_frame_2d
        scan.angle_min       = self.angle_min
        scan.angle_max       = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min       = self.range_min
        scan.range_max       = self.range_max

        num_bins = int((self.angle_max - self.angle_min)/self.angle_increment) + 1
        ranges = [float('inf')] * num_bins

        for x,y,z in pc2.read_points(cloud_b, field_names=("x","y","z"), skip_nans=True):
            if z < self.min_height or z > self.max_height:
                continue
            angle = math.atan2(y,x)
            if not(self.angle_min <= angle <= self.angle_max):
                continue
            r = math.hypot(x,y)
            idx = int((angle - self.angle_min)/self.angle_increment)
            if 0 <= idx < num_bins and r < ranges[idx]:
                ranges[idx] = r

        scan.ranges = ranges
        self.pub_scan.publish(scan)

    def odom_callback(self, msg: Odometry):
        self.odom_counter += 1
        if self.odom_counter % 5 != 0:
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id  = self.odom_frame
        t.child_frame_id   = self.base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation     = msg.pose.pose.orientation
        self.tf_broadcaster1.sendTransform(t)

        # 直接用 Odom(odom->base) * (base->gimbal) 计算 odom->gimbal
        T_odom_base   = pose_to_matrix(msg.pose.pose)         # 4x4
        T_odom_gimbal = T_odom_base @ self.T_base_gimbal      # 4x4

        odom_gimbal = Odometry()
        odom_gimbal.header.stamp    = msg.header.stamp
        odom_gimbal.header.frame_id = self.odom_frame
        odom_gimbal.child_frame_id  = self.gimbal_frame
        odom_gimbal.pose.pose       = matrix_to_pose(T_odom_gimbal)

        # 暂无速度，置零
        odom_gimbal.twist.twist.linear.x  = 0.0
        odom_gimbal.twist.twist.linear.y  = 0.0
        odom_gimbal.twist.twist.linear.z  = 0.0
        odom_gimbal.twist.twist.angular.x = 0.0
        odom_gimbal.twist.twist.angular.y = 0.0
        odom_gimbal.twist.twist.angular.z = 0.0

        self.pub_gimbal_odom.publish(odom_gimbal)

    def odom2d_callback(self, msg: Odometry):
        self.odom2d_counter += 1
        if self.odom2d_counter % 5 != 0:
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id  = self.odom_frame
        t.child_frame_id   = self.base_frame_2d
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation     = msg.pose.pose.orientation
        self.tf_broadcaster2.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)

    # 将 workspace 根目录下的 config.yaml 注入到参数服务器
    config_file = '/home/unitree/ros2_ws/LeggedRobot/src/Ros2Go2Base/config.yaml'
    cli_args = ['--ros-args', '--params-file', config_file]

    node = MessageHandleNode(cli_args=cli_args)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
