# ENPM809y_FinalFall2022

ROS2 Galactic

In this project TurtleBot3 detects a Fiducial marker and moves towards its destination based on the information it receives from the marker. 

## Instructions to run this package

- This project has one launch file which launches the gazebo launch file and all other c++ and python nodes.

- Create a workspace 'ROS2_ws/src'

- Clone all the packages under 'ROS2_ws/src'

- CLI commands:
    ```
    cd ~/ROS2_ws/
    ```
    ```
    colcon build
    ```
    ```
    source install/setup.bash
    ```
    ```
    ros2 launch tb3_bringup final.launch.py
    ```
    ```
    ros2 run target_reacher target_reacher
    ```
