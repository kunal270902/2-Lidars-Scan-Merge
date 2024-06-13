Create workspace in ros2: ros2 pkg create --build-type ament_cmake <package_name>

Clear colcon path: unset COLCON_PREFIX_PATH unset AMENT_PREFIX_PATH unset CMAKE_PREFIX_PATH

To use //Gazebo on Testbed: ros2 launch my_bot launch_sim.launch.py  

Launch my_bot in rviz: rviz2 -d src/my_bot/config/laser_scan.rviz

Launch teleop twist keyboard: ros2 run teleop_twist_keyboard teleop_twist_keyboard

Launch 2 lidar merge script: ros2 launch scan_merger_custom scan_merger_launch.py




