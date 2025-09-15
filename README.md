# CARLA Base Controller

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

This repository provides a collection of ROS packages for controlling a vehicle in the CARLA simulator.

## Project Goal

The primary goal of this project is to develop a robust `base_controller` for an autonomous vehicle in the CARLA simulator. This controller is not intended to be the primary driving system but rather a safety-critical fallback mechanism. In a real-world scenario, a more advanced main controller, such as a Model Predictive Controller (MPC), would handle the primary driving tasks. However, if the main controller malfunctions or fails, the `CARLA_base_controller` is designed to take over, ensuring the vehicle can maintain a safe state, such as following a predefined safe path or coming to a controlled stop. This provides a crucial layer of redundancy and safety for the autonomous driving system.

## Packages & Features

This repository is a ROS workspace containing the following packages:

### 1. `carla_pid_controller`
A simple PID controller to regulate the vehicle's longitudinal speed.

**Usage:**
```bash
rosrun carla_pid_controller pid_controller
```

**Topics:**
-   **Publishes:** `/carla/ego_vehicle/vehicle_control_cmd` (`carla_msgs/CarlaEgoVehicleControl`)
-   **Subscribes:** `/carla/ego_vehicle/vehicle_status` (`carla_msgs/CarlaEgoVehicleStatus`)
-   **Debug Topics:** `/pid_controller/target_speed`, `/pid_controller/current_speed`

**Result:**
![PID Controller Demo](carla_pid_result.gif)

---

### 2. `pure_pursuit_controller`
An implementation of the Pure Pursuit algorithm for lateral control and path tracking. It includes a basic version and an advanced version with more stable PID control for speed.

**Usage:**
```bash
roslaunch pure_pursuit_controller pure_pursuit.launch
```
*Note: The launch file runs the `pure_pursuit_controller_advanced` node by default.*

**Parameters (`pure_pursuit.launch`):**
-   `csv_file`: Path to the CSV file containing the waypoints.
-   `odom_topic`: Vehicle's odometry topic.
-   `control_topic`: Vehicle's control command topic.
-   `wheelbase`: Vehicle's wheelbase.
-   `target_speed`: Target speed for the vehicle.
-   `lookahead_k`: Lookahead gain.
-   `min_lookahead_distance`: Minimum lookahead distance.
-   `pid_kp`, `pid_ki`, `pid_kd`: PID gains for the speed controller.

**Result:**
![Pure Pursuit Demo](carla_pure_pursuit_result.gif)

---

### 3. `velocity_planning`
A node to generate a smooth velocity profile for a given path, considering curvature and acceleration limits.

**Usage:**
```bash
rosrun velocity_planning velocity_planning
```

**Topics:**
-   `/goal_odometry`: Publishes the goal odometry.
-   `/steering_angle`: Publishes the calculated steering angle.
-   `/goal_marker`: Publishes a marker for the goal point in RViz.
-   `/target_speed`: Publishes the target speed.
-   `/path_marker_array`: Publishes markers for the path in RViz.

**Result:**
![Velocity Planning Demo](carla_velocity_planning.gif)

## Prerequisites

-   CARLA Simulator (version 0.9.13)
-   ROS 1 Noetic
-   ROS-Bridge for CARLA
-   `carla-ros-msgs`

## Installation

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/your-username/CARLA_base_controller.git
    ```
2.  **Build the packages:**
    ```bash
    cd CARLA_base_controller
    catkin_make
    ```
3.  **Source the workspace:**
    ```bash
    source devel/setup.bash
    ```

## My Graduation Project

I used the `carla_pid_controller` and `pure_pursuit_controller` packages from this repository for my graduation project. The goal was to implement a self-driving car in the CARLA simulator. The PID controller was used for longitudinal control to maintain a constant speed, and the Pure Pursuit controller was used for lateral control to follow a predefined path. This repository was a great starting point, and I was able to successfully complete my project using it as a foundation.

You can find the final report of this project on here: [![Final_Report](https://github.com/Highsky7/CARLA_base_controller/blob/main/CARLA_base_controller_final_report.pdf)]

## Contributing

Contributions are welcome! Please open an issue or submit a pull request.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
