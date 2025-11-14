## 🔧 Prerequisites

To run this project, you must install **ROS2 Humble** and the required control libraries.  
All dependencies can be installed by executing:

```bash
./setup_ros.sh


colcon build
source install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint
ros2 launch ar ur5_control.launch.py
