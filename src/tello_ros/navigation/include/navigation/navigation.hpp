#ifndef NAVIGATION_NODE_HPP
#define NAVIGATION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tello_msgs/msg/flight_data.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include "tello_msgs/srv/tello_action.hpp"


#include "tf2/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>


#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
// #include <format>/
#include "pid.hpp"
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <cmath>

using namespace std::chrono_literals;

class Navigation : public rclcpp::Node
{
    public:
        Navigation();
    
    private:
        // node parameters
        std::string path_topic_name_;

        bool odom_started = false;
        bool simulation = true;
        int start = 0;


        double x_ = 0;
        double y_ = 0;
        double z_ = 0;

        double roll_;
        double pitch_;
        double yaw_ = 0;

        double qx_;
        double qy_;
        double qz_;
        double qw_;

        PID pid = PID(0.1, 100, -100, 0.1, 0.01, 0.5);
        // ( double dt, double max, double min, double Kp, double Kd, double Ki );

        int index_;
        geometry_msgs::msg::Twist twist_msg;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr path_subscriber_;
        rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr flight_subscriber_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        void pathCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
        void timer_callback();
        void flight_data_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg);

        rclcpp::Client<tello_msgs::srv::TelloAction>::SharedPtr client;
};

#endif // NAVIGATION_NODE_HPP