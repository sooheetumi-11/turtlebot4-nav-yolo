#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Node ROS 2: Điều hướng tự động qua 3 điểm (Nav2) kết hợp nhận diện vật thể thời gian thực bằng YOLOv8.
Tác giả: Robotics Engineer & Computer Vision Specialist
Môi trường: ROS 2 Humble / Gazebo Fortress / TurtleBot 4
"""

import math
import sys
import threading
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.context import Context
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from ultralytics import YOLO


def create_pose_stamped(navigator: BasicNavigator, x: float, y: float, yaw_rad: float) -> PoseStamped:
    """
    Chuyển đổi tọa độ phẳng (x, y, yaw) thành PoseStamped cho Nav2.
    Sử dụng công thức chuyển góc Euler phẳng (Yaw) sang Quaternion (qx=0, qy=0).
    """
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0

    # Quaternion cho chuyển động xoay quanh trục Z:
    # qz = sin(yaw / 2), qw = cos(yaw / 2)
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
    return pose


class YoloCameraNode(Node):
    """
    Node ROS 2 xử lý hình ảnh camera:
    - Chạy trên một ROS Context riêng biệt để tránh xung đột Wait-Set với Nav2 Action Client (Issue #1455).
    - Subscribe topic camera từ TurtleBot 4 (/camera/image hoặc /oakd/rgb/preview/image_raw).
    - Chuyển đổi định dạng ROS Image -> OpenCV BGR.
    - Thực hiện suy luận (inference) bằng YOLOv8 tối ưu tốc độ (Frame skipping giảm tải CPU).
    - Vẽ Bounding Box, nhãn lớp, độ tin cậy.
    - Hiển thị trực tiếp qua cửa sổ OpenCV (cv2.imshow).
    - Publish ảnh kết quả lên topic /yolo/detections_image để xem trên RViz2.
    - In thông tin vật thể phát hiện được ra màn hình Terminal.
    """
    def __init__(self, context: Context = None):
        super().__init__('yolo_camera_node', context=context)

        # Khai báo tham số topic camera đầu vào và topic đầu ra cho RViz2
        self.declare_parameter('camera_topic', '/camera/image')
        self.declare_parameter('output_topic', '/yolo/detections_image')
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.get_logger().info(f'Đang tải mô hình YOLOv8n và kết nối tới topic: {camera_topic}')

        # Tải mô hình YOLOv8 Nano
        self.model = YOLO('yolov8n.pt')
        self.bridge = CvBridge()

        # Publisher để đẩy luồng ảnh đã vẽ bounding box lên RViz2
        self.image_pub = self.create_publisher(Image, output_topic, 10)
        self.get_logger().info(f'Sẽ publish ảnh nhận diện lên topic RViz2: {output_topic}')

        # Cấu hình QoS Best Effort, depth=1 để hình ảnh luôn đạt FPS thực
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            qos
        )

        self.frame_count = 0
        self.last_print_time = 0.0  # Dùng để throttle in thông tin ra terminal
        self.infer_interval = 3     # Chạy YOLO mỗi 3 frames (~10 FPS) để dành CPU cho Gazebo
        self.last_detections = []   # Kết quả nhận diện lần cuối
        self.last_annotated_frame = None  # Frame đã vẽ bbox - tái dùng cho các frame bị skip
        self.get_logger().info('YOLO Camera Node đã sẵn sàng nhận luồng video!')

    def image_callback(self, msg: Image):
        try:
            # Chuyển đổi ROS Image sang ảnh OpenCV BGR
            cv_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as err:
            self.get_logger().error(f'Lỗi CvBridge: {err}')
            return

        self.frame_count += 1
        current_time = time.time()

        # 1. Chạy nhận diện YOLOv8 định kỳ (mỗi 3 frame hoặc frame đầu tiên)
        # Giúp tiết kiệm tối đa CPU cho Gazebo giữ Real-Time Factor (RTF) = 1.0
        if self.frame_count % self.infer_interval == 0 or self.last_annotated_frame is None:
            # Inference TRÊN ĐÚNG frame hiện tại → tọa độ box khớp với ảnh này
            results = self.model(cv_frame, verbose=False, conf=0.35)

            # Trích xuất thông tin các vật thể phát hiện được
            boxes = results[0].boxes
            detections = []
            if boxes is not None and len(boxes) > 0:
                for box in boxes:
                    cls_id = int(box.cls[0])
                    cls_name = self.model.names[cls_id]
                    conf = float(box.conf[0])
                    detections.append((cls_name, conf))
            self.last_detections = detections

            # Vẽ Bounding Box, nhãn lớp và độ tự tin LÊN ĐÚNG FRAME vừa inference
            # Tọa độ box và ảnh khớp nhau hoàn toàn → không bị lệch
            annotated = results[0].plot()

            # Lưu lại frame đã annotate để tái dùng cho các frame bị skip
            self.last_annotated_frame = annotated

            # In thông tin vật thể phát hiện được ra màn hình Terminal (cách nhau 0.8s)
            if detections and (current_time - self.last_print_time >= 0.8):
                obj_str = ", ".join([f"{name} ({conf*100:.1f}%)" for name, conf in detections])
                print(f"[YOLO DETECT] Phát hiện {len(detections)} vật thể: {obj_str}")
                self.last_print_time = current_time
        else:
            # Frame bị skip: Tái dùng ảnh annotate đã render sẵn từ lần inference trước
            # Không vẽ lại tọa độ cũ lên ảnh mới → tuyệt đối không bị lệch
            annotated = self.last_annotated_frame

        # 2. Thêm thông tin frame count và số lượng vật thể lên góc màn hình
        annotated_frame = annotated.copy()
        det_summary = f"Detected: {len(self.last_detections)} obj(s)"
        cv2.putText(
            annotated_frame,
            f'Frame: {self.frame_count} | {det_summary}',
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )

        # 3. Publish ngược lại lên Image topic để xem trực tiếp trên RViz2
        try:
            ros_img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            ros_img_msg.header = msg.header
            self.image_pub.publish(ros_img_msg)
        except CvBridgeError as err:
            self.get_logger().warn(f'Không thể publish ảnh RViz: {err}')

        # 4. Hiển thị cửa sổ trực tiếp bằng OpenCV
        cv2.imshow('TurtleBot 4 - YOLOv8 Live Vision', annotated_frame)

        # waitKey(1) kích hoạt GUI event loop của OpenCV
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            self.get_logger().warn("Nhấn phím 'q': Đang dừng node...")
            if self.context.ok():
                self.context.shutdown()


def navigation_worker(navigator: BasicNavigator, waypoints: list):
    """
    Luồng phụ chuyên trách cho Navigation (Nav2).
    Chạy trên Default ROS Context, hoàn toàn độc lập với Camera Node.
    """
    time.sleep(2.0)  # Chờ hệ thống ổn định
    print('\n' + '='*65)
    print('>>> [Nav2 Worker] Đang kiểm tra trạng thái hoạt động của Nav2 Stack...')
    navigator.waitUntilNav2Active()
    print('>>> [Nav2 Worker] Nav2 đã ACTIVE! Bắt đầu lộ trình tuần tra 3 điểm.')
    print('='*65 + '\n')

    total_goals = len(waypoints)

    for i, (x, y, yaw) in enumerate(waypoints, start=1):
        if not rclpy.ok():
            break

        goal_pose = create_pose_stamped(navigator, x, y, yaw)
        print(f'\n[NAV2 STATUS] >>> Di chuyển đến ĐIỂM {i}/{total_goals}: (x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad)')

        # Gửi mục tiêu đến Nav2
        navigator.goToPose(goal_pose)

        # Theo dõi tiến độ trong khi robot di chuyển
        while not navigator.isTaskComplete():
            if not rclpy.ok():
                navigator.cancelTask()
                return

            feedback = navigator.getFeedback()
            time.sleep(0.5)

        # Kiểm tra kết quả đến đích
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f'✔ [THÀNH CÔNG] Robot đã tới ĐIỂM {i}/{total_goals} an toàn!')
        elif result == TaskResult.CANCELED:
            print(f'⚠ [CẢNH BÁO] Mục tiêu ĐIỂM {i} đã bị hủy bỏ!')
            break
        elif result == TaskResult.FAILED:
            print(f'✖ [THẤT BẠI] Lập quỹ đạo tới ĐIỂM {i} không thành công!')
            break

        # Tạm dừng 2 giây quan sát tại mỗi vị trí
        time.sleep(2.0)

    print('\n' + '='*65)
    print('>>> [Nav2 Worker] Hoàn thành toàn bộ lộ trình tuần tra 3 điểm!')
    print('='*65 + '\n')


def main(args=None):
    # 1. Khởi tạo Default Context cho Navigation (BasicNavigator)
    rclpy.init(args=args)
    navigator = BasicNavigator()

    # 2. Khởi tạo Isolated Context và Executor riêng cho Camera + YOLO
    # Giải pháp cô lập context loại bỏ hoàn toàn xung đột Wait-Set với Nav2 Action Client
    camera_context = Context()
    camera_context.init()

    camera_node = YoloCameraNode(context=camera_context)
    camera_executor = SingleThreadedExecutor(context=camera_context)
    camera_executor.add_node(camera_node)

    # 3 điểm tuần tra: Đặt khoảng cách an toàn (1.2m - 1.5m) so với vật thể
    # để tránh bị vùng Costmap Inflation của Nav2 làm giảm tốc độ tiến
    waypoints = [
        (0.00,  1.50,  1.57),    # Điểm 1: Cách người đứng (y=2.75) 1.25m, tầm nhìn trực diện
        (0.00, -1.20, -1.57),    # Điểm 2: Cách biển báo Stop (y=-2.5) 1.30m
        (2.20, -1.50,  0.00)     # Điểm 3: Cách cái ghế (x=3.7) 1.50m
    ]

    # Khởi động luồng điều hướng Nav2 chạy ngầm (Daemon Thread)
    nav_thread = threading.Thread(
        target=navigation_worker,
        args=(navigator, waypoints),
        daemon=True
    )
    nav_thread.start()

    # Luồng chính (Main Thread) thực thi spin cho camera và xử lý giao diện OpenCV
    try:
        camera_executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print("\nĐang dọn dẹp tài nguyên và tắt hệ thống...")
        if navigator:
            navigator.cancelTask()
            navigator.destroy_node()
        cv2.destroyAllWindows()
        camera_node.destroy_node()
        camera_executor.shutdown()
        if camera_context.ok():
            camera_context.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        print("Đã tắt an toàn!")


if __name__ == '__main__':
    main()
