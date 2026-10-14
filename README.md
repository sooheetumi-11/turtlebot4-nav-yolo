# TurtleBot 4 Autonomous Simulation (ROS 2 Humble & Gazebo Fortress)

Dự án cung cấp gói mô phỏng toàn diện cho robot **TurtleBot 4** (nền tảng di động iRobot® Create® 3 kết hợp tháp cảm biến TurtleBot 4, RPLiDAR A1 và camera OAK-D-PRO) hoạt động trên nền **ROS 2 Humble** và trình mô phỏng **Gazebo Fortress (Ignition Gazebo 6)**. 

Hệ thống hỗ trợ trọn vẹn quy trình tự hành: **Mô phỏng vật lý → Điều khiển thủ công (Teleop) → Quét lập bản đồ (SLAM) → Định vị (AMCL Localization) → Tự hành dẫn đường (Nav2 Autonomous Navigation)**.

---

## 📑 Mục lục
1. [Cấu trúc Repository](#1-cấu-trúc-repository)
2. [Yêu cầu hệ thống](#2-yêu-cầu-hệ-thống)
3. [Cài đặt & Build Workspace](#3-cài-đặt--build-workspace)
4. [Hướng dẫn chạy chi tiết](#4-hướng-dẫn-chạy-chi-tiết)
   - [Chuẩn bị môi trường Terminal](#chuẩn-bị-môi-trường-terminal)
   - [Bước 1: Khởi động mô phỏng (Simulation)](#bước-1-khởi-động-mô-phỏng-simulation)
   - [Bước 2: Quét lập bản đồ (SLAM Mapping)](#bước-2-quét-lập-bản-đồ-slam-mapping)
   - [Bước 3: Định vị robot (Localization)](#bước-3-định-vị-robot-localization)
   - [Bước 4: Tự hành dẫn đường (Navigation)](#bước-4-tự-hành-dẫn-đường-navigation)


---

## 1. Cấu trúc Repository

```text
turtlebot4_ws/src/
├── robot_description/     # Mô hình URDF/Xacro robot, cảm biến và cấu hình mô phỏng
│   ├── urdf/              # Định nghĩa link, joint, plugin Gazebo cho TurtleBot 4 & Create 3
│   ├── meshes/            # File 3D (.dae) thân xe, bánh xe, lidar, camera
│   ├── launch/            # display.launch.py (hiển thị mô hình trên RViz độc lập)
│   ├── config/            # simulation.yaml
│   └── rviz/              # Cấu hình giao diện RViz cho robot description
│
├── robot_sim/             # Môi trường mô phỏng Gazebo Fortress và cầu nối ROS-Gazebo
│   ├── worlds/            # File thế giới SDF: custom.sdf (chữ U), demo.sdf, empty.sdf
│   ├── config/            # bridge_config.yaml (cầu nối topic giữa ROS 2 và Gazebo)
│   ├── launch/            # sim, gazebo, spawn, bridge launch files
│   └── scripts/           # Công cụ sinh world tự động và script smoke test
│
├── robot_node/            # Node xử lý và điều khiển từ bàn phím
│   ├── robot_node/        # teleop_keyboard.py (bộ điều khiển bàn phím an toàn kèm bộ lọc gia tốc)
│   └── test/              # Unit test kiểm tra bộ lọc vận tốc
│
├── robot_mapping/         # Thiết lập quét bản đồ SLAM
│   ├── config/            # slam.yaml (cấu hình thuật toán SLAM Toolbox)
│   ├── launch/            # mapping.launch.py, save_map.launch.py
│   └── rviz/              # Cấu hình giao diện RViz phục vụ mapping
│
└── robot_navigation/      # Ngăn xếp định vị và tự hành Nav2
    ├── maps/              # Bản đồ đã quét được lưu trữ (.yaml và .pgm)
    ├── config/            # nav2.yaml (cấu hình costmap, planner, controller Nav2)
    ├── launch/            # localization.launch.py, navigation.launch.py
    └── rviz/              # Cấu hình giao diện RViz phục vụ navigation
```

| Package | Nhiệm vụ chính |
| :--- | :--- |
| **`robot_description`** | Nạp mô hình hình học robot (URDF/Xacro), mesh 3D, TF tree và thông số cảm biến. |
| **`robot_sim`** | Khởi chạy Gazebo Fortress, nạp bản đồ thế giới (world), spawn robot và kích hoạt `ros_gz_bridge`. |
| **`robot_node`** | Cung cấp node bàn phím tương tác trực tiếp qua `/cmd_vel` với cơ chế timeout an toàn. |
| **`robot_mapping`** | Quét môi trường bằng cảm biến 2D LiDAR với `slam_toolbox` (Online Async SLAM) và lưu bản đồ. |
| **`robot_navigation`** | Thực thi ngăn xếp Nav2: định vị bằng hạt AMCL, lập quỹ đạo toàn cục/cục bộ và né vật cản. |

---

## 2. Yêu cầu hệ thống

- **Hệ điều hành:** Ubuntu 22.04 LTS (Jammy Jellyfish).
- **ROS 2:** Humble Hawksbill.
- **Trình mô phỏng:** Gazebo Fortress (`ign gazebo` / `gz sim` phiên bản 6.x).

Cài đặt các gói phụ thuộc cần thiết bằng lệnh apt:

```bash
sudo apt update
sudo apt install -y git python3-colcon-common-extensions python3-yaml \
  ros-humble-ros-gz \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-teleop-twist-keyboard
```

---

## 3. Cài đặt & Build Workspace

### Bước 3.1: Tạo thư mục workspace và tải mã nguồn

Nếu bạn đang tạo mới workspace từ đầu:

```bash
# Tạo thư mục workspace
mkdir -p ~/workspaces/turtlebot4_ws/src
cd ~/workspaces/turtlebot4_ws/src

# Clone repository về thư mục src
git clone https://github.com/TNSang25/Turtlebot4.git .
```

### Bước 3.2: Build toàn bộ workspace

Đứng tại thư mục gốc của workspace (`~/workspaces/turtlebot4_ws`) và tiến hành build:

```bash
cd ~/workspaces/turtlebot4_ws

# Source môi trường ROS 2 cơ bản
source /opt/ros/humble/setup.bash

# Cài đặt tự động các dependency còn thiếu (nếu có)
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build tất cả các package trong workspace
colcon build --symlink-install
```

> **Lưu ý:** Cờ `--symlink-install` giúp các file script Python, launch và config được tạo liên kết mềm, khi chỉnh sửa không cần phải biên dịch lại.

---

## 4. Hướng dẫn chạy chi tiết

### Chuẩn bị môi trường Terminal

Quy trình vận hành thường sử dụng **3 đến 4 cửa sổ Terminal riêng biệt**. Trước khi gõ bất kỳ lệnh nào trong từng terminal, hãy luôn đảm bảo đã source đầy đủ môi trường:

```bash
cd ~/workspaces/turtlebot4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=69
```

> 💡 **Mẹo:** Bạn có thể thêm các dòng lệnh trên vào cuối file `~/.bashrc` để mọi terminal mới mở đều tự động nhận diện workspace:
> ```bash
> echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
> echo "source ~/workspaces/turtlebot4_ws/install/setup.bash" >> ~/.bashrc
> echo "export ROS_DOMAIN_ID=69" >> ~/.bashrc
> ```

---

### Bước 1: Khởi động mô phỏng (Simulation)

Mở **Terminal 1** và khởi chạy Gazebo cùng TurtleBot 4:

```bash
ros2 launch robot_sim sim.launch.py
```

- Mặc định lệnh trên sẽ mở thế giới **`custom.sdf`** (với chướng ngại vật hình chữ U và các cột trụ để test SLAM/Nav).
- Robot sẽ được sinh tại tọa độ `(x: 0.0, y: 0.0, z: 0.02)`.

**Các tham số tùy chọn:**
| Tham số | Giá trị mặc định | Giải thích |
| :--- | :--- | :--- |
| `world` | `custom` | Tên world (`custom`, `demo`, `empty`) hoặc đường dẫn file `.sdf` bất kỳ |
| `enable_camera`| `true` | Bật/tắt camera RGB-D OAK-D-PRO |
| `headless` | `false` | Chạy chế độ không đồ họa (chỉ chạy core mô phỏng, tiết kiệm RAM/GPU) |
| `rviz` | `false` | Tự động mở cửa sổ RViz hiển thị mô hình robot |
| `x`, `y`, `yaw` | `0.0`, `0.0`, `0.0` | Tọa độ và góc xoay khi xuất hiện của robot |

*Ví dụ khởi chạy thế giới trống (empty) và mở kèm RViz:*
```bash
ros2 launch robot_sim sim.launch.py world:=empty rviz:=true
```

---

### Bước 2: Quét lập bản đồ (SLAM Mapping)

Giữ nguyên **Terminal 1** đang chạy Gazebo.

#### 2.1. Khởi chạy SLAM Toolbox & RViz2
Mở **Terminal 2**:

```bash
ros2 launch robot_mapping mapping.launch.py
```
Cửa sổ RViz2 sẽ hiện ra hiển thị luồng dữ liệu LiDAR 2D (`/scan`) và quá trình sinh bản đồ lưới chiếm dụng (`/map`).

#### 2.2. Điều khiển robot di chuyển để quét
Mở **Terminal 3** và chạy node điều khiển bàn phím tùy chỉnh:

```bash
ros2 run robot_node teleop_keyboard
```

**Bảng phím điều khiển:**
| Phím | Tác vụ | Ghi chú |
| :---: | :--- | :--- |
| `w` / `s` | Tiến thẳng / Lùi thẳng | Vận tốc tuyến tính: ~0.30 m/s |
| `a` / `d` | Quay trái / Quay phải tại chỗ | Vận tốc góc: ~0.80 rad/s |
| `q` / `e` | Tiến cua vòng sang trái / phải | Kết hợp cả tiến và xoay |
| `z` / `c` | Lùi cua vòng sang trái / phải | Kết hợp lùi và xoay |
| `Space` | Dừng khẩn cấp | Phát tín hiệu dừng ngay lập tức |
| `Ctrl + C`| Dừng xe và thoát node | |

Hãy lái robot đi chậm quanh khắp các khu vực trong phòng ảo để cảm biến LiDAR quét trọn vẹn mọi chướng ngại vật và tường chắn.

#### 2.3. Lưu bản đồ đã quét
Khi bản đồ trên RViz đã sắc nét và kín các góc, mở **Terminal 4** để lưu lại bản đồ:

```bash
# Lưu vào đường dẫn maps mặc định của gói robot_navigation
ros2 launch robot_mapping save_map.launch.py \
  map_name:=$PWD/src/robot_navigation/maps/my_custom_map
```

Sau khi hoàn tất, trong thư mục `src/robot_navigation/maps/` sẽ xuất hiện 2 file:
- `my_custom_map.yaml`: Chứa thông số cấu hình bản đồ (resolution, origin, thresholds).
- `my_custom_map.pgm`: Ảnh nhị phân thể hiện không gian trống, vật cản và vùng chưa biết.

Sau khi lưu map xong:
1. Nhấn `Ctrl + C` tại **Terminal 3** để tắt teleop.
2. Nhấn `Ctrl + C` tại **Terminal 2** để tắt mapping.
3. **Vẫn giữ nguyên Terminal 1 (Gazebo) đang chạy.**

---

### Bước 3: Định vị robot (Localization)

Sử dụng thuật toán **AMCL (Adaptive Monte Carlo Localization)** để ước lượng chính xác vị trí robot trên bản đồ đã quét.

Mở **Terminal 2**:

```bash
ros2 launch robot_navigation localization.launch.py \
  map:=$PWD/src/robot_navigation/maps/my_custom_map.yaml
```

**Thao tác đặt vị trí ban đầu (Initial Pose):**
1. Cửa sổ RViz2 sẽ mở ra với bản đồ vừa lưu.
2. Trên thanh công cụ trên cùng của RViz, nhấn nút **`2D Pose Estimate`**.
3. Nhấp chuột vào vị trí tương ứng của robot trên bản đồ (so với vị trí hiện tại trong Gazebo), giữ chuột và kéo mũi tên xanh chỉ theo đúng hướng đầu robot đang quay.
4. Chùm hạt cảm biến (màu xanh lá/đỏ) sẽ hội tụ quanh xe khi tia LiDAR khớp với các bức tường trên map.

**Kiểm tra cây biến đổi tọa độ (TF Tree) (Tùy chọn):**
Mở một terminal khác và gõ:
```bash
ros2 run tf2_ros tf2_echo map base_link
```
Nếu terminal liên tục xuất ra tọa độ Translation & Rotation ổn định thì quá trình định vị đã thành công. Nhấn `Ctrl + C` để đóng lệnh kiểm tra này.

> ⚠️ **LƯU Ý:** **Không được tắt** Terminal 2 này, giữ nguyên để cung cấp vị trí cho bước tiếp theo.

---

### Bước 4: Tự hành dẫn đường (Navigation)

Mở **Terminal 3** và khởi chạy hệ thống điều hướng **Nav2**:

```bash
ros2 launch robot_navigation navigation.launch.py
```

Chờ khoảng 5-10 giây để toàn bộ các plugin costmap, planner, controller và lifecycle nodes chuyển sang trạng thái `active`.

**Thao tác điều hướng trên RViz:**
1. Quay lại cửa sổ RViz đang mở từ bước Localization (hoặc RViz của Navigation).
2. Nhấn vào công cụ **`Nav2 Goal`** trên thanh toolbar đỉnh RViz.
3. Nhấp vào bất kỳ điểm đích nào trong vùng trống trên bản đồ, kéo chuột để chọn hướng quay mong muốn khi tới đích.
4. **Kết quả:**
   - Một đường dẫn toàn cục (Global Plan - nét vàng) sẽ xuất hiện từ robot tới đích.
   - Costmap cục bộ và toàn cục sẽ hiển thị vùng đệm an toàn quanh các bức tường.
   - Robot sẽ tự động tính toán vận tốc, né tránh vật cản và di chuyển mượt mà về đích.
