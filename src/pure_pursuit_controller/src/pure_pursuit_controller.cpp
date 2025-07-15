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
#include <visualization_msgs/Marker.h>  // For visualization markers

class PurePursuitController {
public:
    PurePursuitController(ros::NodeHandle& nh, const std::string& odom_topic, const std::string& control_topic, const std::string& csv_file)
        : nh_(nh), odom_topic_(odom_topic), control_topic_(control_topic), csv_file_(csv_file), target_speed_(15.0) {

        // 퍼블리셔 및 구독자 초기화
        control_pub_ = nh_.advertise<carla_msgs::CarlaEgoVehicleControl>(control_topic_, 10);
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic_, 1, &PurePursuitController::odomCallback, this);

        // 목표 지점을 오도메트리 메시지로 퍼블리시할 퍼블리셔 초기화
        goal_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("goal_odometry", 1);

        // 조향각을 퍼블리시할 퍼블리셔 초기화
        steering_angle_pub_ = nh_.advertise<std_msgs::Float64>("steering_angle", 1);

        // RViz 마커 퍼블리셔 초기화
        goal_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("goal_marker", 1);

        // CSV에서 경로 데이터 읽기
        loadPathFromCSV();
    }

    void run(double rate_hz) {
        ros::Rate rate(rate_hz);
        while (ros::ok()) {
            if (!path_coordinates_.empty()) {
                // 목표 지점 찾기
                geometry_msgs::Point goal_point = findGoalPoint();

                if (isValidGoalPoint(goal_point)) {
                    // 조향각 계산
                    double steering_angle = calculateSteeringAngle(current_position_, goal_point, current_yaw_);

                    // 목표 지점을 오도메트리 메시지로 퍼블리시
                    publishGoalOdometry(goal_point);

                    // 목표 지점을 RViz에 마커로 퍼블리시
                    publishGoalMarker(goal_point);

                    // 차량 제어 명령 발행
                    publishControl(steering_angle);
                }
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher control_pub_; // 차량 제어 명령 퍼블리셔
    ros::Subscriber odom_sub_;
    ros::Publisher goal_odom_pub_;  // 목표 지점 오도메트리 퍼블리셔
    ros::Publisher steering_angle_pub_;  // 조향각 퍼블리셔
    ros::Publisher goal_marker_pub_;  // RViz 마커 퍼블리셔

    std::string odom_topic_;
    std::string control_topic_;
    std::string csv_file_; //csv_file 변수

    std::vector<geometry_msgs::Point> path_coordinates_;
    nav_msgs::Odometry current_odom_;
    geometry_msgs::Point current_position_;
    double current_yaw_;
    double current_speed_;
    const double target_speed_;  // 목표 속도 (m/s)

    const double L_ = 3.0;  // 차량의 wheelbase, 단위: 미터
    const double look_ahead_distance_ = 6.0;  // 전방주시거리를 6m로 고정

    // Odometry 데이터 콜백 함수
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_odom_ = *msg;
        geometry_msgs::Point vehicle_center_position = current_odom_.pose.pose.position;

        // Quaternion을 이용해 차량의 yaw 값을 계산
        tf::Quaternion q(
            current_odom_.pose.pose.orientation.x,
            current_odom_.pose.pose.orientation.y,
            current_odom_.pose.pose.orientation.z,
            current_odom_.pose.pose.orientation.w);
        current_yaw_ = tf::getYaw(q);  // 간결하게 yaw 값만 추출

        // yaw 방향으로 휠베이스 절반을 차량 중심에서 뒤로 이동시켜 후륜축 좌표 계산(현재 좌표 기준을 후륜축 중점으로 설정)
        current_position_.x = vehicle_center_position.x - (L_ / 2.0) * cos(current_yaw_);
        current_position_.y = vehicle_center_position.y - (L_ / 2.0) * sin(current_yaw_);
        current_position_.z = vehicle_center_position.z;  // z축 좌표는 그대로 유지

        // 차량의 속도 계산 (m/s)
        current_speed_ = sqrt(pow(current_odom_.twist.twist.linear.x, 2) +
                              pow(current_odom_.twist.twist.linear.y, 2)); // 차량의 절대속도 계산(z좌표는 고려 x)
    }

    // CSV 파일에서 경로 데이터를 읽어오는 함수
    void loadPathFromCSV() {
        std::ifstream file(csv_file_);
        // 파일이 제대로 열렸는지 확인
        if (!file.is_open()) {
            std::cerr << "Error: Cannot open file " << csv_file_ << std::endl;
            return;
        }

        std::string line;

        // 첫 번째 줄(헤더)을 읽어 무시합니다.
        std::getline(file, line);

        // 파일의 줄 수를 먼저 계산하여 벡터의 공간을 미리 할당합니다.
        size_t line_count = 0;
        while (std::getline(file, line)) {
        ++line_count;
        }
        file.clear();  // 파일 스트림 상태 리셋
        file.seekg(0); // 파일 포인터를 처음으로 이동

        // 벡터 크기를 미리 할당
        path_coordinates_.reserve(line_count - 1);  // 첫 줄은 헤더이므로 제외

        // 다시 첫 번째 줄(헤더)을 무시
        std::getline(file, line);

        // CSV 파일에서 경로 좌표 읽기
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string item;
            geometry_msgs::Point point;

            // CSV 항목 분리
            std::getline(ss, item, ',');  // %time (무시)
            std::getline(ss, item, ',');  // field.header.seq (무시)
            std::getline(ss, item, ',');  // field.header.stamp (무시)
            std::getline(ss, item, ',');  // field.header.frame_id (무시)
            std::getline(ss, item, ',');  // field.child_frame_id (무시)

            // 포지션 좌표
            std::getline(ss, item, ',');
            point.x = std::stod(item);
            std::getline(ss, item, ',');
            point.y = std::stod(item);
            std::getline(ss, item, ',');
            point.z = std::stod(item);

            // Orientation (무시)
            for (int i = 0; i < 4; ++i) std::getline(ss, item, ',');

            // Covariance (무시)
            for (int i = 0; i < 36; ++i) std::getline(ss, item, ',');

            path_coordinates_.emplace_back(point);
        }
        file.close();
    }

    // 목표 지점을 경로에서 찾는 함수
    geometry_msgs::Point findGoalPoint() {
        geometry_msgs::Point goal_point;
        double min_distance = std::numeric_limits<double>::max();
        double goal_direction = current_yaw_;  // 현재 차량의 yaw 방향

        for (const auto& point : path_coordinates_) {
            double distance = sqrt(pow(point.x - current_position_.x, 2) + pow(point.y - current_position_.y, 2));

            // 목표 지점이 현재 위치의 앞쪽에 있는지 확인
            double angle_to_goal = atan2(point.y - current_position_.y, point.x - current_position_.x);
            double angle_diff = angle_to_goal - goal_direction;

            // 각도를 -π ~ π 사이로 정규화
            while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
            while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

            // 목표 지점이 유효하고, 전방 주시 거리 이상인지 확인
            if (distance >= look_ahead_distance_ &&
                distance < min_distance &&
                fabs(angle_diff) < M_PI/2) {  // 차량의 방향과 일치하는 지점만 선택
                goal_point = point;
                min_distance = distance;
            }
        }
        return goal_point;
    }

    // 목표 지점이 유효한지 확인하는 함수
    bool isValidGoalPoint(const geometry_msgs::Point& goal_point) {
        double distance = sqrt(pow(goal_point.x - current_position_.x, 2) + pow(goal_point.y - current_position_.y, 2));
        return distance >= look_ahead_distance_;  // 너무 가까운 목표 지점은 무시
    }

    // 조향각 계산 함수
    double calculateSteeringAngle(const geometry_msgs::Point& current_position,
                                  const geometry_msgs::Point& goal_position,
                                  double yaw) {
        double dx = goal_position.x - current_position.x;
        double dy = goal_position.y - current_position.y;
        double goal_direction = atan2(dy, dx);
        double alpha = goal_direction - yaw;


        // 조향각 계산
        double steering_angle = atan2(2.0 * L_ * sin(-alpha), look_ahead_distance_);
        double steering_angle_deg = steering_angle*(180.0/M_PI);
        ROS_INFO_STREAM("Steering Angle: "<<steering_angle_deg);
        return steering_angle;
    }
    // 속도에 따라 throttle 값을 동적으로 계산하는 함수
    double calculateThrottle() {
        double speed_error = target_speed_ - current_speed_;
        double throttle = 0.3 + (0.7 * speed_error);  // 비례 제어
        return std::min(std::max(throttle, 0.0), 1.0);  // 0~1 범위로 제한
    }

    // 차량 제어 명령을 발행하는 함수
    void publishControl(double steering_angle) {
    carla_msgs::CarlaEgoVehicleControl control_cmd;

    // 조향각을 -1 ~ 1 범위로 변환 (최대 조향각을 35도로 가정)
    const double max_steering_angle_rad = 35.0 * (M_PI / 180.0);  // 최대 조향각 35도 -> 라디안으로 변환
    double normalized_steering_angle = steering_angle / max_steering_angle_rad;

    // 정규화된 값을 -1 ~ 1 범위로 제한
    control_cmd.steer = std::max(-0.7, std::min(0.7, normalized_steering_angle));

    // 동적으로 계산된 throttle
    control_cmd.throttle = calculateThrottle();

    control_pub_.publish(control_cmd);

    // 조향각 퍼블리시 (디버그용)
    std_msgs::Float64 steering_msg;
    steering_msg.data = control_cmd.steer;
    steering_angle_pub_.publish(steering_msg);
}

    // 목표 지점을 오도메트리 메시지로 퍼블리시하는 함수
    void publishGoalOdometry(const geometry_msgs::Point& goal_point) {
        nav_msgs::Odometry goal_odom;
        goal_odom.header.stamp = ros::Time::now();
        goal_odom.header.frame_id = "map";  // 좌표계 설정
        goal_odom.pose.pose.position = goal_point;
        goal_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);  // 방향은 무시

        goal_odom_pub_.publish(goal_odom);
    }

    // RViz에 목표 지점을 마커로 퍼블리시하는 함수
    void publishGoalMarker(const geometry_msgs::Point& goal_point) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "goal_marker";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = goal_point;
        marker.pose.orientation.w = 1.0;  // 회전 없음
        marker.scale.x = 1.0;  // 구의 크기
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.r = 1.0;  // 빨간색
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;  // 불투명

        goal_marker_pub_.publish(marker);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pure_pursuit_controller");
    ros::NodeHandle nh;

    // 사용자가 지정한 파라미터
    std::string odom_topic = "/carla/ego_vehicle/odometry";  // 오도메트리 토픽
    std::string control_topic = "/carla/ego_vehicle/vehicle_control_cmd";  // 차량 제어 명령 토픽
    std::string csv_file = "/home/highsky/odometry_data2.csv";  // CSV 파일 경로

    PurePursuitController controller(nh, odom_topic, control_topic, csv_file);
    controller.run(50.0);  // 주기: 50 Hz

    return 0;
}
