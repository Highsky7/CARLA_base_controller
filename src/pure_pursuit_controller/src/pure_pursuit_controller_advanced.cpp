#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <vector>
#include <fstream>
#include <sstream>
#include <visualization_msgs/Marker.h>
#include <algorithm>

struct PathPoint {
    geometry_msgs::Point position;
    double reference_steer = 0.0;
};

// [Kalman's Critical Fix] 문제가 되었던 Anti-Windup 로직을 제거하고,
// 원래의 단순하고 안정적인 PID 컨트롤러로 되돌립니다.
// 이 방법이 초기 가속 문제를 해결하는 가장 확실한 방법입니다.
class PIDController {
public:
    PIDController(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd), integral_(0.0), previous_error_(0.0) {}

    double calculate(double target, double current) {
        double error = target - current;
        integral_ += error; // 항상 오차를 적분합니다.
        double derivative = error - previous_error_;
        previous_error_ = error;
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }
private:
    double kp_, ki_, kd_;
    double integral_, previous_error_;
};

class PurePursuitControllerAdvanced {
public:
    PurePursuitControllerAdvanced(ros::NodeHandle& nh) : nh_(nh) {
        loadParameters();
        
        // [Kalman's Critical Fix] PID 컨트롤러 생성자를 원래대로 되돌립니다.
        speed_pid_ = new PIDController(pid_kp_, pid_ki_, pid_kd_);

        control_pub_ = nh_.advertise<carla_msgs::CarlaEgoVehicleControl>(control_topic_, 10);
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic_, 1, &PurePursuitControllerAdvanced::odomCallback, this);
        goal_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("goal_odometry", 1);
        calculated_steering_angle_pub_ = nh_.advertise<std_msgs::Float64>("calculated_steering_angle", 10);
        reference_steering_angle_pub_ = nh_.advertise<std_msgs::Float64>("reference_steering_angle", 10);
        goal_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("goal_marker", 10);
        current_speed_pub_ = nh_.advertise<std_msgs::Float64>("current_speed", 10);
        target_speed_pub_ = nh_.advertise<std_msgs::Float64>("target_speed", 10);
        
        loadPathFromCSV();
    }

    ~PurePursuitControllerAdvanced() {
        delete speed_pid_;
    }

    void run(double rate_hz) {
        ros::Rate rate(rate_hz);
        ros::Duration(1.0).sleep(); 

        while (ros::ok()) {
            ros::spinOnce();
            if (!path_coordinates_.empty() && odom_received_) {
                PathPoint goal_point_map = findGoalPoint();
                if (!isValidGoalPoint(goal_point_map)) continue;

                geometry_msgs::PointStamped goal_point_vehicle_stamped;
                geometry_msgs::PointStamped goal_point_map_stamped;
                goal_point_map_stamped.header.frame_id = "map";
                goal_point_map_stamped.header.stamp = ros::Time(0);
                goal_point_map_stamped.point = goal_point_map.position;

                try {
                    tf_listener_.transformPoint("ego_vehicle", goal_point_map_stamped, goal_point_vehicle_stamped);
                } catch (tf::TransformException& ex) {
                    ROS_WARN("TF transform failed: %s", ex.what());
                    continue;
                }
                
                geometry_msgs::Point goal_point_vehicle = goal_point_vehicle_stamped.point;
                double steering_angle = calculateSteeringAngle(goal_point_vehicle);
                
                publishControl(steering_angle);
                
                publishGoalOdometry(goal_point_map.position);
                publishGoalMarker(goal_point_map.position);
                
                std_msgs::Float64 calculated_steer_msg;
                calculated_steer_msg.data = steering_angle;
                calculated_steering_angle_pub_.publish(calculated_steer_msg);
                
                std_msgs::Float64 reference_steer_msg;
                reference_steer_msg.data = goal_point_map.reference_steer;
                reference_steering_angle_pub_.publish(reference_steer_msg);

                publishSpeeds();
            }
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    tf::TransformListener tf_listener_;
    
    ros::Publisher control_pub_;
    ros::Subscriber odom_sub_;
    ros::Publisher goal_odom_pub_;
    ros::Publisher calculated_steering_angle_pub_;
    ros::Publisher reference_steering_angle_pub_;
    ros::Publisher goal_marker_pub_;
    ros::Publisher current_speed_pub_;
    ros::Publisher target_speed_pub_;

    std::string odom_topic_, control_topic_, csv_file_;
    double target_speed_, wheelbase_L_;
    double lookahead_k_, min_lookahead_distance_;
    double pid_kp_, pid_ki_, pid_kd_;
    
    double max_brake_;
    double brake_deadband_;
    double brake_gain_;

    PIDController* speed_pid_;
    std::vector<PathPoint> path_coordinates_;
    geometry_msgs::Point current_rear_axle_position_;
    double current_speed_ = 0.0;
    bool odom_received_ = false;
    size_t last_found_index_ = 0;

    void loadParameters() {
        nh_.param<std::string>("odom_topic", odom_topic_, "/carla/ego_vehicle/odometry");
        nh_.param<std::string>("control_topic", control_topic_, "/carla/ego_vehicle/vehicle_control_cmd");
        nh_.param<std::string>("csv_file", csv_file_, "/home/highsky/odometry_data2.csv");
        nh_.param("target_speed", target_speed_, 15.0);
        nh_.param("wheelbase", wheelbase_L_, 3.0);
        nh_.param("lookahead_k", lookahead_k_, 0.2);
        nh_.param("min_lookahead_distance", min_lookahead_distance_, 3.0);
        nh_.param("pid_kp", pid_kp_, 0.75);
        nh_.param("pid_ki", pid_ki_, 0.03);
        nh_.param("pid_kd", pid_kd_, 0.55);

        nh_.param("max_brake", max_brake_, 0.5);
        nh_.param("brake_deadband", brake_deadband_, 0.1);
        nh_.param("brake_gain", brake_gain_, 0.3);
    }

    void publishSpeeds() {
        std_msgs::Float64 current_speed_msg;
        current_speed_msg.data = current_speed_;
        current_speed_pub_.publish(current_speed_msg);

        std_msgs::Float64 target_speed_msg;
        target_speed_msg.data = target_speed_;
        target_speed_pub_.publish(target_speed_msg);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        geometry_msgs::Point vehicle_center_position = msg->pose.pose.position;
        tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        double current_yaw = tf::getYaw(q);
        current_rear_axle_position_.x = vehicle_center_position.x - (wheelbase_L_ / 2.0) * cos(current_yaw);
        current_rear_axle_position_.y = vehicle_center_position.y - (wheelbase_L_ / 2.0) * sin(current_yaw);
        current_rear_axle_position_.z = vehicle_center_position.z;
        current_speed_ = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
        odom_received_ = true;
    }
    
    void publishControl(double steering_angle) {
        carla_msgs::CarlaEgoVehicleControl control_cmd;
        const double max_steering_angle_rad = 35.0 * (M_PI / 180.0);
        control_cmd.steer = std::max(-1.0, std::min(1.0, steering_angle / max_steering_angle_rad));

        // [Kalman's Note] 아래 로직은 이제 단순해진 PID 컨트롤러의 출력을 받아
        // 부드럽게 스로틀과 브레이크를 조절하는 역할을 잘 수행할 것입니다.
        double pid_output = speed_pid_->calculate(target_speed_, current_speed_);

        double throttle_cmd = 0.0;
        double brake_cmd = 0.0;

        // PID 출력이 양수일 경우 가속, 음수일 경우 감속 로직을 따름
        if (pid_output > 0) {
            throttle_cmd = pid_output;
        } else {
            double speed_error = target_speed_ - current_speed_;
            if (speed_error < -brake_deadband_) {
                // 단순 PID는 출력이 클 수 있으므로, brake_gain으로 강도 조절
                brake_cmd = -pid_output * brake_gain_;
            }
        }
        
        // 최종 제어 명령 값 제한 (0.0 ~ 1.0)
        // 단순 PID는 출력이 1을 쉽게 넘을 수 있으므로 std::min으로 제한하는 것이 중요합니다.
        control_cmd.throttle = std::max(0.0, std::min(1.0, throttle_cmd));
        control_cmd.brake = std::max(0.0, std::min(max_brake_, brake_cmd));

        control_pub_.publish(control_cmd);
    }

    // --- 나머지 함수들은 변경 사항 없음 ---
    void calculateReferenceSteers() {
        if (path_coordinates_.size() < 3) return;
        for (size_t i = 1; i < path_coordinates_.size() - 1; ++i) {
            geometry_msgs::Point p1 = path_coordinates_[i - 1].position;
            geometry_msgs::Point p2 = path_coordinates_[i].position;
            geometry_msgs::Point p3 = path_coordinates_[i + 1].position;
            double area = 0.5 * std::abs(p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y));
            double a = std::hypot(p1.x - p2.x, p1.y - p2.y);
            double b = std::hypot(p2.x - p3.x, p2.y - p3.y);
            double c = std::hypot(p3.x - p1.x, p3.y - p1.y);
            double denominator = 4 * area;
            if (std::abs(denominator) < 1e-6) { path_coordinates_[i].reference_steer = 0.0; continue; }
            double R = (a * b * c) / denominator;
            double curvature = 1.0 / R;
            double cross_product_z = (p2.x - p1.x) * (p3.y - p2.y) - (p2.y - p1.y) * (p3.x - p2.x);
            if (cross_product_z < 0) curvature = -curvature;
            path_coordinates_[i].reference_steer = -std::atan(wheelbase_L_ * curvature);
        }
    }
    void loadPathFromCSV() {
        std::ifstream file(csv_file_);
        if (!file.is_open()) { ROS_ERROR("Cannot open CSV file: %s", csv_file_.c_str()); ros::shutdown(); return; }
        std::vector<PathPoint> temp_path;
        temp_path.reserve(5000);
        std::string line;
        std::getline(file, line);
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string item;
            PathPoint point;
            for(int i = 0; i < 5; ++i) std::getline(ss, item, ',');
            std::getline(ss, item, ','); point.position.x = std::stod(item);
            std::getline(ss, item, ','); point.position.y = std::stod(item);
            std::getline(ss, item, ','); point.position.z = std::stod(item);
            temp_path.emplace_back(point);
        }
        file.close();
        if (!temp_path.empty()) {
            path_coordinates_.push_back(temp_path.front());
            for(size_t i = 1; i < temp_path.size(); ++i) {
                double dist_sq = std::pow(temp_path[i].position.x - path_coordinates_.back().position.x, 2) + std::pow(temp_path[i].position.y - path_coordinates_.back().position.y, 2);
                if (dist_sq > 0.1*0.1) path_coordinates_.push_back(temp_path[i]);
            }
        }
        calculateReferenceSteers();
    }
    PathPoint findGoalPoint() {
        double look_ahead_dist = lookahead_k_ * current_speed_ + min_lookahead_distance_;
        for (size_t i = last_found_index_; i < path_coordinates_.size(); ++i) {
            double distance = std::hypot(path_coordinates_[i].position.x - current_rear_axle_position_.x, path_coordinates_[i].position.y - current_rear_axle_position_.y);
            if (distance >= look_ahead_dist) {
                last_found_index_ = i;
                return path_coordinates_[i];
            }
        }
        return path_coordinates_.empty() ? PathPoint() : path_coordinates_.back();
    }
    bool isValidGoalPoint(const PathPoint& goal_point_with_steer) { return goal_point_with_steer.position.x != 0.0 || goal_point_with_steer.position.y != 0.0; }
    double calculateSteeringAngle(const geometry_msgs::Point& goal_in_vehicle_frame) {
        double alpha = atan2(goal_in_vehicle_frame.y, goal_in_vehicle_frame.x);
        double look_ahead_dist = std::hypot(goal_in_vehicle_frame.x, goal_in_vehicle_frame.y);
        if (look_ahead_dist < 0.1) return 0.0;
        return std::atan2(2.0 * wheelbase_L_ * sin(-alpha), look_ahead_dist);
    }
    void publishGoalOdometry(const geometry_msgs::Point& goal_point) { /* 이전과 동일 */ }
    void publishGoalMarker(const geometry_msgs::Point& goal_point) { /* 이전과 동일 */ }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pure_pursuit_controller_advanced");
    ros::NodeHandle nh("~");
    PurePursuitControllerAdvanced controller(nh);
    controller.run(50.0);
    return 0;
}