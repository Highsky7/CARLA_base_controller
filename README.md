# CARLA Base Controller

This repository provides a collection of ROS packages for controlling a vehicle in the [CARLA simulator](http://carla.org/). It includes implementations of a PID controller for longitudinal control and Pure Pursuit controllers for lateral control, along with a velocity planning module.

## Project Goal

The primary goal of this project is to develop a robust `base_controller` for an autonomous vehicle in the CARLA simulator. This controller is not intended to be the primary driving system but rather a safety-critical fallback mechanism. In a real-world scenario, a more advanced main controller, such as a Model Predictive Controller (MPC), would handle the primary driving tasks. However, if the main controller malfunctions or fails, the `CARLA_base_controller` is designed to take over, ensuring the vehicle can maintain a safe state, such as following a predefined safe path or coming to a controlled stop. This provides a crucial layer of redundancy and safety for the autonomous driving system.

## Features

- **PID Controller:** A simple PID controller to regulate the vehicle's speed.
- **Pure Pursuit Controller:** A basic implementation of the Pure Pursuit algorithm for path tracking.
- **Advanced Pure Pursuit Controller:** An enhanced version of the Pure Pursuit controller with improved stability and parameter handling.
- **Velocity Planning:** A module to generate a velocity profile for a given path, considering curvature and acceleration limits.

## Prerequisites

- CARLA Simulator (version 0.9.13)
- ROS 1 Noetic
- ROS-Bridge for CARLA
- `carla-ros-msgs`

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

## Usage

### PID Controller

The PID controller can be used to maintain a constant speed.

-   **To run:**
    ```bash
    rosrun carla_pid_controller pid_controller
    ```
-   **Topics:**
    -   `/carla/ego_vehicle/vehicle_control_cmd`: Publishes `carla_msgs/CarlaEgoVehicleControl` messages.
    -   `/carla/ego_vehicle/vehicle_status`: Subscribes to `carla_msgs/CarlaEgoVehicleStatus` messages.
    -   `/pid_controller/target_speed`: Publishes the target speed.
    -   `/pid_controller/current_speed`: Publishes the current speed.

### Pure Pursuit Controller

The Pure Pursuit controller follows a predefined path.

-   **To run:**
    ```bash
    roslaunch pure_pursuit_controller pure_pursuit.launch
    ```
-   **Parameters:**
    -   `csv_file`: Path to the CSV file containing the waypoints.
    -   `odom_topic`: Topic for the vehicle's odometry.
    -   `control_topic`: Topic for the vehicle's control commands.
    -   `wheelbase`: Vehicle's wheelbase.
    -   `target_speed`: Target speed for the vehicle.
    -   `lookahead_k`: Lookahead gain.
    -   `min_lookahead_distance`: Minimum lookahead distance.
    -   `pid_kp`, `pid_ki`, `pid_kd`: PID gains for the speed controller.

### Velocity Planning

The velocity planning node generates a velocity profile for a given path.

-   **To run:**
    ```bash
    rosrun velocity_planning velocity_planning
    ```
-   **Topics:**
    -   `/goal_odometry`: Publishes the goal odometry.
    -   `/steering_angle`: Publishes the calculated steering angle.
    -   `/goal_marker`: Publishes a marker for the goal point in RViz.
    -   `/target_speed`: Publishes the target speed.
    -   `/path_marker_array`: Publishes markers for the path in RViz.

## Graduation Project

I used the `carla_pid_controller` and `pure_pursuit_controller` packages from this repository for my graduation project. The goal of my project was to implement a self-driving car in the CARLA simulator. The PID controller was used for longitudinal control to maintain a constant speed, and the Pure Pursuit controller was used for lateral control to follow a predefined path. This repository was a great starting point for my project, and I was able to successfully complete my project.

## Contributing

Contributions are welcome! Please open an issue or submit a pull request.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.