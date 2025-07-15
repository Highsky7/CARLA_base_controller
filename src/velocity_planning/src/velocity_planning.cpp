#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <vector>
#include <fstream>
#include <sstream>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <memory>
#include <algorithm>
#include <limits>
#include <iomanip>
class VelocityPlanning {
public:
    VelocityPlanning(const std::vector<geometry_msgs::Point>& path, double max_lateral_accel = 3.0, double min_longitudinal_accel = -3.0, double max_longitudinal_accel = 3.0, double min_speed = 15.0, double max_speed = 35.0)
        : path_(path), max_lateral_accel_(max_lateral_accel), min_longitudinal_accel_(min_longitudinal_accel), max_longitudinal_accel_(max_longitudinal_accel), min_speed_(min_speed), max_speed_(max_speed) {}

    std::vector<double> planVelocities() {
        std::vector<double> velocities(path_.size(), max_speed_);
        velocities[0] = min_speed_;  // Start speed

        for (size_t i = 1; i < path_.size() - 1; ++i) {
            double curvature = calculateCurvature(path_[i - 1], path_[i], path_[i + 1]);
            double max_curvature_speed = (curvature > 0) ? std::sqrt(max_lateral_accel_ / curvature) : max_speed_;
            double distance_to_next = distance(path_[i - 1], path_[i]);

            double planned_speed = std::min({max_curvature_speed, velocities[i - 1] + max_longitudinal_accel_ * distance_to_next, max_speed_});
            velocities[i] = std::max(planned_speed, min_speed_);
        }

        smoothVelocityTransitions(velocities);
        return velocities;
    }

private:
    std::vector<geometry_msgs::Point> path_;
    double max_lateral_accel_, min_longitudinal_accel_, max_longitudinal_accel_, min_speed_, max_speed_;

    double calculateCurvature(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, const geometry_msgs::Point& p3) {
        double a = distance(p2, p3);
        double b = distance(p1, p3);
        double c = distance(p1, p2);
        double s = (a + b + c) / 2.0;
        double area = std::sqrt(std::max(0.0, s * (s - a) * (s - b) * (s - c)));
        if (a * b * c == 0) return 0.0;
        return (4.0 * area) / (a * b * c);
    }

    double distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    }

    void smoothVelocityTransitions(std::vector<double>& velocities) {
        for (size_t i = 1; i < velocities.size(); ++i) {
            double distance_to_next = distance(path_[i - 1], path_[i]);
            double max_acc_speed = std::sqrt(std::max(0.0, velocities[i - 1] * velocities[i - 1] + 2 * max_longitudinal_accel_ * distance_to_next));
            velocities[i] = std::min(velocities[i], max_acc_speed);
        }

        for (size_t i = velocities.size() - 2; i > 0; --i) {
            double distance_to_next = distance(path_[i], path_[i + 1]);
            double max_dec_speed = std::sqrt(std::max(0.0, velocities[i + 1] * velocities[i + 1] + 2 * std::abs(min_longitudinal_accel_) * distance_to_next));
            velocities[i] = std::min(velocities[i], max_dec_speed);
        }
    }
};

class PurePursuitController {
public:
    PurePursuitController(ros::NodeHandle& nh, const std::string& odom_topic, const std::string& control_topic, const std::string& csv_file)
        : nh_(nh), odom_topic_(odom_topic), control_topic_(control_topic), csv_file_(csv_file), L_(3.0) {
        control_pub_ = nh_.advertise<carla_msgs::CarlaEgoVehicleControl>(control_topic_, 10);
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic_, 1, &PurePursuitController::odomCallback, this);
        goal_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("goal_odometry", 1);
        steering_angle_pub_ = nh_.advertise<std_msgs::Float64>("steering_angle", 1);
        goal_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("goal_marker", 1);
        target_speed_pub_ = nh_.advertise<std_msgs::Float64>("target_speed", 1);
        path_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("path_marker_array", 1);  // 경로 마커 발행

        loadPathFromCSV();
        publishPathMarkers();  // 경로를 초기화와 동시에 시각화

        velocity_planning_ = std::make_unique<VelocityPlanning>(path_coordinates_);
        planned_velocities_ = velocity_planning_->planVelocities();

        // Save velocities with timestamps for PlotJuggler compatibility
        saveVelocitiesToCSV("Velocity_Plan.csv", planned_velocities_, 1.0 / 50.0); // Assuming 50Hz rate, so timestep is 1/50 seconds
    }

    void run(double rate_hz) {
        ros::Rate rate(rate_hz);
        while (ros::ok()) {
            if (!path_coordinates_.empty()) {
                size_t goal_index;
                geometry_msgs::Point goal_point = findGoalPoint(goal_index);
                if (isValidGoalPoint(goal_point)) {
                    double steering_angle = calculateSteeringAngle(current_position_, goal_point, current_yaw_);
                    double throttle, brake;
                    calculateControlInputs(goal_index, throttle, brake);
                    publishGoalOdometry(goal_point);
                    publishGoalMarker(goal_point);
                    publishControl(steering_angle, throttle, brake);
                }
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher control_pub_, goal_odom_pub_, steering_angle_pub_, goal_marker_pub_, target_speed_pub_, path_marker_pub_;
    ros::Subscriber odom_sub_;
    std::string odom_topic_, control_topic_, csv_file_;
    std::vector<geometry_msgs::Point> path_coordinates_;
    std::vector<double> planned_velocities_;
    nav_msgs::Odometry current_odom_;
    geometry_msgs::Point current_position_;
    double current_yaw_, current_speed_, L_, look_ahead_distance_;
    double previous_speed_ = 0.0;
    std::unique_ptr<VelocityPlanning> velocity_planning_;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_odom_ = *msg;
        tf::Quaternion q(current_odom_.pose.pose.orientation.x, current_odom_.pose.pose.orientation.y, current_odom_.pose.pose.orientation.z, current_odom_.pose.pose.orientation.w);
        current_yaw_ = tf::getYaw(q);
        current_position_.x = current_odom_.pose.pose.position.x - (L_ / 2.0) * cos(current_yaw_);
        current_position_.y = current_odom_.pose.pose.position.y - (L_ / 2.0) * sin(current_yaw_);
        current_position_.z = current_odom_.pose.pose.position.z;
        current_speed_ = sqrt(pow(current_odom_.twist.twist.linear.x, 2) + pow(current_odom_.twist.twist.linear.y, 2));
        updateLookAheadDistance();
    }

    void updateLookAheadDistance() {
        look_ahead_distance_ = 3.0 + (current_speed_ * 0.5);
    }

    void loadPathFromCSV() {
        std::ifstream file(csv_file_);
        if (!file.is_open()) {
            std::cerr << "Error: Cannot open file " << csv_file_ << std::endl;
            return;
        }
        std::string line;
        std::getline(file, line);
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string item;
            geometry_msgs::Point point;
            std::getline(ss, item, ',');
            std::getline(ss, item, ',');
            std::getline(ss, item, ',');
            std::getline(ss, item, ',');
            std::getline(ss, item, ',');
            std::getline(ss, item, ',');
            point.x = std::stod(item);
            std::getline(ss, item, ',');
            point.y = std::stod(item);
            std::getline(ss, item, ',');
            point.z = std::stod(item);
            path_coordinates_.emplace_back(point);
        }
        file.close();
    }

    void saveVelocitiesToCSV(const std::string& filename, const std::vector<double>& velocities, double time_step) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open file " << filename << std::endl;
        return;
    }
    
    // Write header for PlotJuggler
    file << "Time,Velocity\n";

    // Initialize time variable and write each time-velocity pair to CSV
    double time = 0.0;
    for (const auto& velocity : velocities) {
        // Use fixed-point notation with 2 decimal places for time
        file << std::fixed << std::setprecision(2) << time << "," << velocity << "\n";
        
        // Increment time by fixed time step
        time += time_step;
    }

    file.close();
    std::cout << "Velocity plan saved to " << filename << std::endl;
}

    geometry_msgs::Point findGoalPoint(size_t& index) {
        geometry_msgs::Point goal_point;
        double min_distance = std::numeric_limits<double>::max();
        double goal_direction = current_yaw_;
        for (size_t i = 0; i < path_coordinates_.size(); ++i) {
            const auto& point = path_coordinates_[i];
            double distance = sqrt(pow(point.x - current_position_.x, 2) + pow(point.y - current_position_.y, 2));
            double angle_to_goal = atan2(point.y - current_position_.y, point.x - current_position_.x);
            double angle_diff = angle_to_goal - goal_direction;
            angle_diff = std::remainder(angle_diff, 2.0 * M_PI);
            if (distance >= look_ahead_distance_ && distance < min_distance && fabs(angle_diff) < M_PI / 2) {
                goal_point = point;
                min_distance = distance;
                index = i;
            }
        }
        return goal_point;
    }

    bool isValidGoalPoint(const geometry_msgs::Point& goal_point) {
        double distance = sqrt(pow(goal_point.x - current_position_.x, 2) + pow(goal_point.y - current_position_.y, 2));
        return distance >= look_ahead_distance_;
    }

    double calculateSteeringAngle(const geometry_msgs::Point& current_position, const geometry_msgs::Point& goal_position, double yaw) {
        double dx = goal_position.x - current_position.x;
        double dy = goal_position.y - current_position.y;
        double goal_direction = atan2(dy, dx);
        double alpha = goal_direction - yaw;
        double steering_angle = atan2(2.0 * L_ * sin(-alpha), look_ahead_distance_);
        return steering_angle;
    }

    void calculateControlInputs(size_t goal_index, double& throttle, double& brake) {
        double target_speed = planned_velocities_[goal_index];
        double speed_error = target_speed - current_speed_;
        throttle = std::min(std::max(0.3 + (0.7 * speed_error), 0.0), 1.0);
        brake = 0.0;
        if (speed_error < 0) {
            brake = std::min(std::abs(speed_error) / 5.0, 1.0);
        }
        previous_speed_ = current_speed_;
        std_msgs::Float64 target_speed_msg;
        target_speed_msg.data = target_speed;
        target_speed_pub_.publish(target_speed_msg);
    }

    void publishControl(double steering_angle, double throttle, double brake) {
        carla_msgs::CarlaEgoVehicleControl control_cmd;
        control_cmd.steer = steering_angle / (35.0 * (M_PI / 180.0));
        control_cmd.throttle = throttle;
        control_cmd.brake = brake;
        control_pub_.publish(control_cmd);
        std_msgs::Float64 steering_msg;
        steering_msg.data = steering_angle;
        steering_angle_pub_.publish(steering_msg);
    }

    void publishGoalOdometry(const geometry_msgs::Point& goal_point) {
        nav_msgs::Odometry goal_odom;
        goal_odom.header.stamp = ros::Time::now();
        goal_odom.header.frame_id = "map";
        goal_odom.pose.pose.position = goal_point;
        goal_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
        goal_odom_pub_.publish(goal_odom);
    }

    void publishGoalMarker(const geometry_msgs::Point& goal_point) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "goal_marker";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = goal_point;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        goal_marker_pub_.publish(marker);
    }

// 경로 시각화 함수 추가
    void publishPathMarkers() {
        visualization_msgs::MarkerArray marker_array;
        for (size_t i = 0; i < path_coordinates_.size(); ++i) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "path_marker";
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position = path_coordinates_[i];
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker_array.markers.push_back(marker);
        }
        path_marker_pub_.publish(marker_array);
    }


};
int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_planning");
    ros::NodeHandle nh;
    std::string odom_topic = "/carla/ego_vehicle/odometry";
    std::string control_topic = "/carla/ego_vehicle/vehicle_control_cmd";
    std::string csv_file = "/home/highsky/odometry_data2.csv";
    PurePursuitController controller(nh, odom_topic, control_topic, csv_file);
    controller.run(50.0);
    return 0;
}
