You need to install ROS2 Humble to run all these commands, you also need corresponding control library, please run "setup_ros.sh" to install all the dependencies we need.

please run:
  colcon build  
  source install/setup.bash 
before running any command.

please run:
'''bash
  ros2 run ros_tcp_endpoint default_server_endpoint

This is used for connecting with Unity, you can not connect with Unity through ROS2 without this command.

please run:
  ros2 launch ar ur5_control.launch.py 
to open the gazebo simulator, make sure there is no error reported in terminal, and terminal continuously show the command sent from the Unity. If everything is correct, you can use control the UR5 in simulator by keyboard(in Unity).
