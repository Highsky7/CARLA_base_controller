#include <ros/ros.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <std_msgs/Float64.h>
#include <rosgraph_msgs/Clock.h>

class PIDController {
public:
    PIDController() : nh_(), target_speed_(10.0), kp_(0.75), ki_(0.03), kd_(0.55),
                      current_speed_(0.0), integral_(0.0), previous_error_(0.0), previous_time_(ros::Time::now()) {
        // Initialize publishers and subscribers
        control_pub_ = nh_.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 10);
        speed_sub_ = nh_.subscribe("/carla/ego_vehicle/vehicle_status", 10, &PIDController::speedCallback, this);
        clock_sub_ = nh_.subscribe("/carla/ego_vehicle/vehicle_control_cmd/clock", 10, &PIDController::clockCallback, this);
        target_speed_pub_ = nh_.advertise<std_msgs::Float64>("/pid_controller/target_speed", 10);
        current_speed_pub_ = nh_.advertise<std_msgs::Float64>("/pid_controller/current_speed", 10);
    }

    void run() {
        ros::Rate rate(20);  // Set loop rate to 20Hz

        while (ros::ok()) {
            ros::spinOnce();  // Process callbacks

            // Debugging: Print current speed in main loop
            ROS_INFO_STREAM("Main Loop - Current Speed: " << current_speed_);

            // Calculate throttle using PID control
            double throttle = calculateThrottle();

            // Publish control message
            publishControlMessage(throttle);

            // Publish target speed
            publishTargetSpeed();

            rate.sleep();  // Maintain loop rate
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher control_pub_;
    ros::Subscriber speed_sub_;
    ros::Subscriber clock_sub_;
    ros::Publisher target_speed_pub_;
    ros::Publisher current_speed_pub_;

    double target_speed_;  // Target speed (m/s)
    double kp_, ki_, kd_;  // PID gain values
    double current_speed_;  // Current speed of the vehicle
    double integral_;  // Integral term
    double previous_error_;  // Previous error for derivative calculation
    ros::Time previous_time_;  // Previous time for time step calculation

    // Callback to update current speed based on vehicle status
    void speedCallback(const carla_msgs::CarlaEgoVehicleStatus::ConstPtr& msg) {
        current_speed_ = msg->velocity;  // Update current speed
        ROS_INFO_STREAM("Current Speed (Callback): " << current_speed_);  // Debug: Print current speed

        // Publish current speed
        std_msgs::Float64 current_speed_msg;
        current_speed_msg.data = current_speed_;
        current_speed_pub_.publish(current_speed_msg);
    }

    // Callback to synchronize simulation time
    void clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg) {
        ros::Time::setNow(msg->clock);  // Update ROS time to match simulation time
        ROS_DEBUG_STREAM("Sim Time Updated: " << msg->clock);  // Debug: Print simulation time
    }

    // Function to calculate throttle value using PID control
    double calculateThrottle() {
        ros::Time current_time = ros::Time::now();

        // If previous time is not set, initialize it
        if (previous_time_.isZero()) {
            previous_time_ = current_time;
            return 0.0;  // Set throttle to 0 initially
        }

        // Calculate time step
        double dt = (current_time - previous_time_).toSec();
        if (dt <= 0.0) {
            ROS_WARN("Time step (dt) is zero or negative.");  // Warn if time step is invalid
            return 0.0;
        }

        // PID control calculations
        double error = target_speed_ - current_speed_;
        integral_ += error * dt;
        double derivative = (error - previous_error_) / dt;

        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

        // Update previous values
        previous_error_ = error;
        previous_time_ = current_time;

        // Limit throttle value between 0 and 1
        if (output < 0.0) output = 0.0;
        if (output > 1.0) output = 1.0;

        ROS_INFO_STREAM("Throttle Output: " << output);  // Debug: Print throttle output
        return output;
    }

    // Function to publish vehicle control message
    void publishControlMessage(double throttle) {
        carla_msgs::CarlaEgoVehicleControl control_msg;
        control_msg.throttle = throttle;
        control_msg.steer = 0.0;  // No steering for straight driving
        control_msg.brake = 0.0;  // No brake
        control_msg.hand_brake = false;
        control_msg.reverse = false;
        control_msg.manual_gear_shift = false;

        control_pub_.publish(control_msg);  // Publish control command
    }

    // Function to publish target speed
    void publishTargetSpeed() {
        std_msgs::Float64 target_speed_msg;
        target_speed_msg.data = target_speed_;
        target_speed_pub_.publish(target_speed_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pid_controller");

    PIDController controller;  // Instantiate PID controller class
    controller.run();  // Run the controller

    return 0;
}
