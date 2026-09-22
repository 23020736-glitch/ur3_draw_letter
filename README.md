# UR3 Trajectory Planning - Draw Letter "Đ" (ROS 2 Humble & MoveIt 2)

Dự án lập kế hoạch quỹ đạo cho tay máy UR3 / UR3e vẽ chữ **Đ** trong môi trường mô phỏng ROS 2 Humble và MoveIt 2.

##  Yêu cầu hệ thống
* Ubuntu 22.04 LTS
* ROS 2 Humble
* MoveIt 2 / ur_moveit_config

##  Cài đặt và Biên dịch
```bash
cd ~/chud_ws
colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash
## Hướng dẫn chạy chương trình
ros2 launch ur3_draw_letter draw_d.launch.py
## Cấu hình RViz
Tại mục Displays -> Chọn Marker -> Thiết lập Topic thành /visualization_marker.
EOF
