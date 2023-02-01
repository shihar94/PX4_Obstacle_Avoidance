#pragma once 


#include "perception_px4/CNN_out.h"
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <iostream>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include "StateHandler.h"
#include <nav_msgs/Odometry.h>
#include <math.h>



class px4_control{

public:
    ros::Subscriber cnn_sub;
    ros::Subscriber state_sub;
    ros::Publisher vel_pub;
    ros::Subscriber odom_sub;
    ros::Timer timer1;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::Publisher set_vel_pub;
    ros::Time last_request;

    mavros_msgs::State current_state;
    mavros_msgs::PositionTarget pos;
    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::SetMode offb_set_mode;
    nav_msgs::Odometry odom_msg;

    float proability_of_collision_=0.0;
    float steering_angle_=0.0;
    double max_forward_index_ = 1.3;
    double desired_forward_velocity_=max_forward_index_;
    double desired_angular_velocity_;
    double alpha_velocity_ = 0.7;
    double alpha_yaw_ = 0.7;
    double yaw_angle = 0.0;
    double critical_prob_coll_= 1.3;
    double yaw_rate;
    double theta,phi;
    double vz_msg,z_msg;
    double x,y,z,w;
    mavros_msgs::PositionTarget pos_msg;
    
public:
    px4_control(ros::NodeHandle& nh);
    void state_callback(const mavros_msgs::State::ConstPtr& msg);
    void deepNetworkCallback(const perception_px4::CNN_out::ConstPtr& msg);
    void callback1(const ros::TimerEvent& event);
    void velocity_publisher();
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    double altitudePID();
    double forward_velocity_calculator(float &proability_of_collision_);
    double angular_velocity_calculator(float &steering_angle_);
    void bitmask_set();
};


