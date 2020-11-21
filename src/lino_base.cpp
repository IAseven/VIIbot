#include "lino_base.h"

LinoBase::LinoBase():
    linear_velocity_x_(0),
    linear_velocity_y_(0),
    angular_velocity_z_(0),
    last_vel_time_(0),
    vel_dt_(0),
    x_pos_(0),
    y_pos_(0),
    heading_(0)
{
    odom_publisher_ = nh_.advertise<nav_msgs::Odometry>("raw_odom", 50);
    velocity_subscriber_ = nh_.subscribe("raw_vel", 50, &LinoBase::velCallback, this);
}

// raw_vel 的回调函数，用来发布 raw_odom 消息
void LinoBase::velCallback(const lino_msgs::Velocities& vel)
{
    ros::Time current_time = ros::Time::now(); //接收到raw_vel的时间

    linear_velocity_x_ = vel.linear_x; //当前x速度
    linear_velocity_y_ = vel.linear_y; //当前y速度
    angular_velocity_z_ = vel.angular_z; //当前z角速度

    vel_dt_ = (current_time - last_vel_time_).toSec();
    last_vel_time_ = current_time;

    double delta_heading = angular_velocity_z_ * vel_dt_; //radians（航向角）
    double delta_x = (linear_velocity_x_ * cos(heading_) - linear_velocity_y_ * sin(heading_)) * vel_dt_; //m，x轴位置
    double delta_y = (linear_velocity_x_ * sin(heading_) + linear_velocity_y_ * cos(heading_)) * vel_dt_; //m，y轴位置

    //calculate current position of the robot（通过累加（积分）来估算机器人的位置和航向角）
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;

    //calculate robot's heading in quaternion angle （将航向角通过ROS自带tf库函数转换为四元数输出）
    //ROS has a function to calculate yaw in quaternion angle
    odom_quat.setRPY(0,0,heading_); //将欧拉角转化为四元数

    odom.header.stamp = current_time; //发布此条消息的时间戳
    odom.header.frame_id = "odom"; //里程计坐标系的id
    odom.child_frame_id = "base_footprint"; //机器人底盘坐标系的id

    //robot's position in x,y, and z （机器人位置）
    odom.pose.pose.position.x = x_pos_;
    odom.pose.pose.position.y = y_pos_;
    odom.pose.pose.position.z = 0.0; //只考虑在二位平面内，所以z轴高度始终为0
    //robot's heading in quaternion （四元数类型的位姿）
    odom.pose.pose.orientation.x = odom_quat.x();
    odom.pose.pose.orientation.y = odom_quat.y();
    odom.pose.pose.orientation.z = odom_quat.z();
    odom.pose.pose.orientation.w = odom_quat.w();
    // 原始里程计的协方差，用于右面数据融合时使用
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[35] = 0.001;

    //linear speed from encoders （速度信息）
    odom.twist.twist.linear.x = linear_velocity_x_;
    odom.twist.twist.linear.y = linear_velocity_y_;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    //angular speed from encoders （角速度）
    odom.twist.twist.angular.z = angular_velocity_z_;
    //速度信息的协方差
    odom.twist.covariance[0] = 0.0001;
    odom.twist.covariance[7] = 0.0001;
    odom.twist.covariance[35] = 0.0001;

    odom_publisher_.publish(odom);
}
