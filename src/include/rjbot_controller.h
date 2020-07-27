# included files to rjbot_controller
#ifndef RJBOT_BASE_H
#define RJBOT_BASE_H

#include <ros/ros.h>
#include <lino_msgs/Velocities.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <stdio.h>
#include <cmath>

class RJbotBase
{
public:
    RJbotBase();
    //void velCallback(const lino_msgs::Velocities& vel);
    // void velCallback( const geometry_msgs::Vector3Stamped& speed)

private:
  //  ros::NodeHandle nh_;
 //   ros::Publisher odom_publisher_;
 //   ros::Subscriber velocity_subscriber_;
    //tf2_ros::TransformBroadcaster odom_broadcaster_;
 //   tf2::Quaternion odom_quat;
 //   geometry_msgs::TransformStamped odom_trans;
//    nav_msgs::Odometry odom;

  ros::NodeHandle n;
  ros::NodeHandle nh_private_("~");
  ros::Subscriber sub = n.subscribe("vel", 50, velCallback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster broadcaster;  
    
 //   float steering_angle_;
    float linear_velocity_x_;
    float linear_velocity_y_;
    float angular_velocity_z_;
    ros::Time last_vel_time_;
    float vel_dt_;
    float x_pos_;
    float y_pos_;
    float heading_;
};

#endif
