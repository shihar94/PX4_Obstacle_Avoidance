#include "px4_control.h"


px4_control::px4_control(ros::NodeHandle& nh){
    state_sub= nh.subscribe<mavros_msgs::State>("mavros/state", 10, &px4_control::state_callback,this);
    cnn_sub = nh.subscribe("/cnn_predictions", 1, &px4_control::deepNetworkCallback,this);
    vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel",10,this);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming",this);
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode",this);
    set_vel_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10,this);
    odom_sub = nh.subscribe<nav_msgs::Odometry>("mavros/local_position/odom",10,&px4_control::odom_callback,this);
    timer1  = nh.createTimer(ros::Duration(.05),&px4_control::callback1 ,this);
    
    ros::Rate rate(30.0);
    while(ros::ok() && !current_state.connected){
        ros::spinOnce(); 
        rate.sleep();
    }

    desired_angular_velocity_ = 0.0;
    //bitmask_set();
    //pos_msg.position.z = 1.5;
    arm_cmd.request.value = true;
    offb_set_mode.request.custom_mode="OFFBOARD";
     while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                    
                }
                last_request = ros::Time::now();
            }
        }
        //set_vel_pub.publish(pos_msg);
    ros::spinOnce();
    rate.sleep();
    }
    
}

void px4_control::state_callback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;

}


void px4_control::deepNetworkCallback(const perception_px4::CNN_out::ConstPtr& msg){
    proability_of_collision_ = msg->collision_prob;
    steering_angle_ = msg->steering_angle;
    // Output modulation
    if (steering_angle_ < -1.0) { steering_angle_ = -1.0;}
    if (steering_angle_ > 1.0) { steering_angle_ = 1.0;}
}

void px4_control::odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
    z_msg = msg->pose.pose.position.z;
    vz_msg = msg->twist.twist.linear.z;
    x = msg->pose.pose.orientation.x;
    y = msg->pose.pose.orientation.y;
    z = msg->pose.pose.orientation.z;
    w = msg->pose.pose.orientation.w;

}



void px4_control::callback1(const ros::TimerEvent& event){
    
    desired_forward_velocity_ = forward_velocity_calculator(proability_of_collision_);
    desired_angular_velocity_ = (1.0 - alpha_yaw_) * desired_angular_velocity_ + alpha_yaw_ * steering_angle_;
    double t1 =  2*(w*y-z*x);
    double t2 =  1-2*(y*y+z*z);
    double yaw_angle = atan2(t1,t2);
    std::cout<<"yaw_angle: "<<yaw_angle*180/3.14<<std::endl;
    double vx = desired_forward_velocity_;//*cos(yaw_angle);
    double vy = desired_forward_velocity_*sin(yaw_angle);
    std::cout<<"Vx: "<<vx<<std::endl;
    std::cout<<"Vy: "<<vy<<std::endl;

    
    geometry_msgs::TwistStamped cmd_msg;
     cmd_msg.twist.linear.z  =altitudePID();
    cmd_msg.twist.linear.x = vx;//desired_forward_velocity_;
    cmd_msg.twist.linear.y = 0.0;//vy;
   if(1-proability_of_collision_>0.5){
        cmd_msg.twist.angular.z =0.0;
   }else {
            cmd_msg.twist.angular.z = desired_angular_velocity_;    
        
        
    }
    cmd_msg.twist.angular.z = 0.0;//desired_angular_velocity_;    
    cmd_msg.twist.linear.z  = altitudePID();

    
    vel_pub.publish(cmd_msg);
    std::cout<<" "<<std::endl;
    std::cout<<"Desired Forward Velocity : "<< desired_forward_velocity_<<std::endl;
    std::cout<<"Probability of Collission: "<<proability_of_collision_<<std::endl;
    std::cout<<"Steering Angle: "<<steering_angle_<<std::endl;
    std::cout<<"Desired Angular Velocity: "<<desired_angular_velocity_<<std::endl;

}




double px4_control::forward_velocity_calculator(float &proability_of_collision_){
    double desired_forward_velocity_m = (1.0 -  proability_of_collision_) * max_forward_index_;
      if (desired_forward_velocity_m <= 0.0)
    {
      ROS_INFO("Detected negative forward velocity! Drone will now stop!");
      desired_forward_velocity_m  = 0;
    }

    // Low pass filter the velocity and integrate it to get the position
    desired_forward_velocity_ = (1.0 - alpha_velocity_) * desired_forward_velocity_
        + alpha_velocity_ * desired_forward_velocity_m;

    if (desired_forward_velocity_ < ((1 - critical_prob_coll_) * max_forward_index_))
    {
      desired_forward_velocity_ = 0.0;
    }



    return desired_forward_velocity_;
}



double px4_control::altitudePID(){
    double kp,kv,zP;
    zP = 1.5;
    kp = 1.4;
    kv = 1.4;

    //get odom info
   


    double z_vel = (zP-z_msg)*kp+kv*(-vz_msg);
    return z_vel;
}

void px4_control::bitmask_set(){
    pos.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;


    pos.type_mask =  mavros_msgs::PositionTarget::IGNORE_PX  |mavros_msgs::PositionTarget::IGNORE_PY |mavros_msgs::PositionTarget::IGNORE_VZ
                    |mavros_msgs::PositionTarget::IGNORE_AFX |mavros_msgs::PositionTarget::IGNORE_AFY 
                    |mavros_msgs::PositionTarget::IGNORE_AFZ //|mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
                    |mavros_msgs::PositionTarget::IGNORE_YAW ;
}