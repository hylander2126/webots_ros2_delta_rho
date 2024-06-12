# webots_ros2_delta_rho
Webots simulation for ros2 of delta_rho holonomic robot platform

*** ROS2 HUBMLE ***

## To Run:

1. Navigate to your ros2 workspace, src, and package folder:

   ```
   (if you don't have a workspace setup yet)
   $ mkdir -p ros2_ws/src/webots_ros2_delta_rho
   
   $ cd ros2_ws/src/webots_ros2_delta_rho
   ```

2. Clone this repository into the new package folder.
3. Source your ros2 distribution & update ros2 dependencies:

   ```
   $ source /opt/ros/humble/setup.bash
   $ rosdep update
   $ rosdep install --from-paths src --ignore-src -r -y
   ```

4. Build your workspace (you should be one level above your src/ directory) and source your new workspace:

   ```
   $ colcon build
   $ source install/setup.bash
   ```

5. Launch the simulation!

  ```
  $ ros2 launch delta_rho_sim webots_sim_launch.py
  ```


For questions or concerns, send an email to Steven Hyland: smhyland@wpi.edu
