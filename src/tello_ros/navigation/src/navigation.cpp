#include "navigation/navigation.hpp"
// #include <tf2_eigen/tf2_eigen.hpp>

Navigation::Navigation() : Node("navigation")
{
    index_ = 0;

    declare_parameter<std::string>("topics.path_topic_name", "path_topic_name_");

    path_topic_name_ = get_parameter("topics.path_topic_name").as_string();
 

    path_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>("poses3d", 1, std::bind(&Navigation::pathCallback, this, std::placeholders::_1));
    flight_subscriber_ = this->create_subscription<tello_msgs::msg::FlightData>("flight_data", 1, std::bind(&Navigation::flight_data_callback, this, std::placeholders::_1));
    // subscriber for odometry ?

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
}


void Navigation::pathCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Path Callback !!!");
    // following_path_ = std::move(msg);
    // path_received_ = true;
    RCLCPP_ERROR(this->get_logger(), "PATH RECEIVED !!!"); 
    // relative pose to target 
    double pose_x = msg->poses[0].position.x - this->x_;
    double pose_y = msg->poses[0].position.y - this->y_;
    tf2::Quaternion q_ = tf2::Quaternion(msg->poses[0].orientation.x, msg->poses[0].orientation.y, msg->poses[0].orientation.z, msg->poses[0].orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_).getRPY(roll, pitch, yaw, 1);
    RCLCPP_ERROR_STREAM(this->get_logger(),"yaw = :" << yaw);

    double pose_yaw = yaw - this->yaw_;

    double k = 0.7;
    double v = k * sqrt(pow(pose_x,2)+ pow(pose_y,2));

    // zbiegajace pole
    double x1 = - pose_x - v;//- 0.70* norm(q.x(), q.y());
    double y1 = - pose_y;

    // proste pole 
    double x2 = - pose_x ;
    double y2 = 0.0;

    double scaling_factor = 2.0;

    double scale =  v*scaling_factor;
    if(scale < 0){scale =0;}
    if(scale > 1){scale =1;}
    // im mniejsza odleglosc tym bardziej proste pole 
    // in wiekszy kat tym badziej zakzywione pole <-----------
    RCLCPP_INFO_STREAM(this->get_logger(), "scale: "<< scale );

    double d_norm = (x1*cos(pose_yaw)+y1*sin(pose_yaw))/sqrt(x1*x1+y1*y1);
    double dhx = -cos(pose_yaw)-k*d_norm;
    double dhy = -sin(pose_yaw);

    // nav_field.msg->h.x = x*scale + x2*(1.0 -scale);
    // nav_field.msg->h.y = y*scale + y2*(1.0 -scale);
    // nav_field.msg->dh.x = (scale - 1)*cos(q.theta()) +q.x()*scaling_factor*d_norm + scale*dhx + x*scaling_factor*d_norm;
    // nav_field.msg->dh.y = scale*dhy + y*scaling_factor*d_norm;
    //vector sterujacy potem przepisuje,y na predkosci 

    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.angular.z = 0.0;
    cmd_vel_pub_->publish(twist)

}
void Navigation::flight_data_callback(const tello_msgs::msg::FlightData::SharedPtr msg)
{
    // calculate pose base on flight data, starts in 0,0,0
    RCLCPP_ERROR(this->get_logger(), "flight data callback !!!");
    rclcpp::Time now = this->get_clock()->now();
    double time_diff = (now - rclcpp::Time(msg->header.stamp)).nanoseconds()/1e9;

    this->roll_= msg->roll;
    this->pitch_= msg->pitch;
    this->yaw_= msg->yaw;

    this->x_ += msg->vgx*cos(this->yaw_)*time_diff - msg->vgy*sin(this->yaw_)*time_diff;
    this->y_ += msg->vgx*sin(this->yaw_)*time_diff + msg->vgy*cos(this->yaw_)*time_diff;



}
// odometry callback{
//     code here 
// }
