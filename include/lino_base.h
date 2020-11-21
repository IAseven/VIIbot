#ifndef LINO_BASE_H
#define LINO_BASE_H

#include <ros/ros.h>
#include <lino_msgs/Velocities.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

// LinoBase 头文件和类的定义

class LinoBase
{
public:
    LinoBase();
    void velCallback(const lino_msgs::Velocities& vel); // 订阅下位机原始速度话题(/raw_vel)的回调函数

private:
    ros::NodeHandle nh_;
    ros::Publisher odom_publisher_; // 原始里程计消息 /raw_odom 的发布者
    ros::Subscriber velocity_subscriber_; // 原始下位机速度消息话题的订阅者
    tf2::Quaternion odom_quat; // 四元数对象，用来描述小车的姿态
    geometry_msgs::TransformStamped odom_trans; // 坐标静态变换对象
    nav_msgs::Odometry odom; // 待发布的里程计消息对象

    float steering_angle_; // ACKERMANN类型小车的转向角
    float linear_velocity_x_; // x轴线速度 m/s
    float linear_velocity_y_; // y轴线速度 m/s
    float angular_velocity_z_; // z轴角速度 rad/s
    ros::Time last_vel_time_; // 最后一次发布raw_odom的时间
    float vel_dt_; // 接收到相邻两次速度的时间差
    float x_pos_; // x轴上的位置 单位mm
    float y_pos_; // y轴上的位置 m
    float heading_; // raw_odom 消息头
};

#endif
