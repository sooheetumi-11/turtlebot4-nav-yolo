# Mô phỏng TurtleBot 4, điều hướng Nav2 và nhận diện YOLO

Workspace ROS 2 Humble mô phỏng TurtleBot 4 trên Gazebo Fortress, hỗ trợ tạo hoặc nạp bản đồ, định vị bằng AMCL, điều hướng bằng Nav2 và nhận diện vật thể từ camera bằng YOLOv8.

## Cấu trúc repository

| Package | Chức năng |
| --- | --- |
| `robot_description` | Mô hình Xacro TurtleBot 4 / Create 3, mesh, frame cảm biến và plugin mô phỏng. |
| `robot_sim` | World Gazebo Fortress, tạo robot và bridge topic giữa ROS với Gazebo. World `custom` có các vật thể thuộc COCO để thử YOLO. |
| `robot_node` | Điều khiển robot bằng bàn phím và node `nav2_yolo_patrol` để đi qua ba điểm trong khi nhận diện ảnh camera. |
| `robot_mapping` | Launch/configuration cho SLAM Toolbox và lưu bản đồ. |
| `robot_navigation` | Tham số Nav2, launch AMCL/Navigation, cấu hình RViz và các bản đồ mẫu. |

Các bản đồ có sẵn:

- `robot_navigation/maps/my_custom_map.yaml` và ảnh bản đồ `my_custom_map.pgm`.
- `robot_navigation/maps/my_map.yaml` và ảnh bản đồ `my_map.pgm`.

Đường dẫn ảnh trong mỗi file YAML là đường dẫn tương đối tính từ thư mục chứa YAML đó.

## Yêu cầu

- Ubuntu 22.04 và ROS 2 Humble.
- Gazebo Fortress và ROS–Gazebo integration.
- Nav2, SLAM Toolbox, Xacro, RViz và các dependency của mô hình TurtleBot.
- OpenCV cho Python, `cv_bridge` và Ultralytics YOLO.

Cài đặt các gói ROS và công cụ cần thiết:

```bash
sudo apt update
sudo apt install -y git python3-pip python3-venv python3-opencv \
  python3-colcon-common-extensions python3-yaml \
  ros-humble-ros-gz ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
  ros-humble-rviz2 ros-humble-slam-toolbox \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-simple-commander ros-humble-cv-bridge
```

## Clone và build workspace

Thư mục gốc repository này được dùng làm thư mục `src` của workspace:

```bash
mkdir -p ~/workspaces/turtlebot4_ws/src
cd ~/workspaces/turtlebot4_ws/src
git clone https://github.com/sooheetumi-11/turtlebot4_simulation.git 

cd ~/workspaces/turtlebot4_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Tạo môi trường Python ảo có thể sử dụng các gói Python ROS đã cài bằng apt, sau đó cài Ultralytics. Hãy kích hoạt môi trường này trong terminal chạy node YOLO:

```bash
cd ~/workspaces/turtlebot4_ws
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install 'numpy<2' ultralytics
```

Trong mỗi terminal chạy ROS, source ROS và workspace. Trong terminal chạy node nhận diện, kích hoạt thêm môi trường `.venv`:

```bash
cd ~/workspaces/turtlebot4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=69
# Chỉ chạy trong terminal khởi chạy node YOLO:
source .venv/bin/activate
```

## Khởi chạy mô phỏng, định vị và điều hướng

Mở các terminal riêng cho từng launch. Giữ Gazebo và localization chạy trong suốt quá trình Nav2 điều khiển robot.

### Terminal 1 — Gazebo và robot

```bash
ros2 launch robot_sim sim.launch.py world:=custom enable_camera:=true
```

World `custom` tham chiếu ba mô hình trên Open Robotics Fuel: người đứng, biển báo dừng và ghế. Gazebo cần kết nối mạng để tải các mô hình lần đầu; sau đó chúng được lưu trong cache Fuel.

### Terminal 2 — Map server, AMCL và RViz

```bash
ros2 launch robot_navigation localization.launch.py \
  map:=$PWD/src/robot_navigation/maps/my_custom_map.yaml
```

Trong RViz, chọn **2D Pose Estimate**, click gần vị trí robot trên bản đồ rồi kéo mũi tên để đặt đúng hướng robot. Chờ scan LiDAR khớp với bản đồ và cụm hạt AMCL ổn định. Giữ launch này chạy.

### Terminal 3 — Nav2

```bash
ros2 launch robot_navigation navigation.launch.py
```

Chờ các lifecycle node của Nav2 chuyển sang trạng thái `active` trước khi chạy tuần tra. Trong workspace này, localization và navigation được khởi chạy riêng: localization cung cấp `/map` và TF `map → odom`; Nav2 lập kế hoạch và điều khiển robot.

## Chạy tuần tra ba điểm và nhận diện YOLO

### Terminal 4 — Node điều hướng và nhận diện

```bash
cd ~/workspaces/turtlebot4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source .venv/bin/activate
export ROS_DOMAIN_ID=69
ros2 launch robot_node nav2_yolo_patrol.launch.py
```

Launch mặc định nhận ảnh từ `/camera/image` — topic camera mà bridge của repository này phát — và xuất ảnh đã vẽ kết quả lên `/yolo/detections_image`. Nếu camera của bạn dùng topic khác, truyền tên topic khi chạy:

```bash
ros2 launch robot_node nav2_yolo_patrol.launch.py \
  camera_topic:=/oakd/rgb/preview/image_raw
```

Node tải mô hình `yolov8n.pt` (Ultralytics tự tải nếu file chưa có), hiển thị ảnh kèm bounding box trong cửa sổ OpenCV và publish ảnh kết quả để xem trong RViz. Nhấn `q` trên cửa sổ ảnh hoặc `Ctrl+C` trong terminal để dừng node.

### Cấu hình các điểm tuần tra

Sửa danh sách `waypoints` trong `robot_node/robot_node/nav2_yolo_patrol.py`. Mỗi điểm có dạng `(x, y, yaw)` và phải nằm trong frame `map`. Các tọa độ mẫu hiện tại tương ứng với vị trí các vật thể trong `robot_sim/worlds/custom.sdf`:

```python
waypoints = [
    (0.00,  1.50,  1.57),
    (0.00, -1.20, -1.57),
    (2.20, -1.50,  0.00),
]
```

Dùng công cụ **Nav2 Goal** trên RViz và xem topic `/goal_pose` để lấy cả vị trí lẫn hướng. `/clicked_point` chỉ chứa tọa độ điểm, không có hướng. Nếu workspace được build mà không dùng `--symlink-install`, hãy build lại sau khi sửa code.


## Các giao diện ROS chính

- `/scan`, `/odom`, `/tf`, `/clock`: dữ liệu mô phỏng dùng cho định vị và điều hướng.
- `/map`, `/initialpose`, `/amcl_pose`: bản đồ và các giao diện định vị AMCL.
- `/navigate_to_pose`: action Nav2 mà node tuần tra dùng để lần lượt tới từng goal.
- `/cmd_vel`: lệnh vận tốc Nav2 gửi tới robot qua bridge Gazebo.
- `/camera/image`: ảnh camera đầu vào của cấu hình repository này.
- `/yolo/detections_image`: ảnh đã được YOLO vẽ kết quả nhận diện.

Không chạy điều khiển bàn phím cùng lúc với Nav2, vì cả hai đều có thể gửi lệnh vận tốc lên `/cmd_vel`.
