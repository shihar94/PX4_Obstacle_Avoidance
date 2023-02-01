#ifndef STATEHANDLER_H
#define STATEHANDLER_H

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <string> 

class StateHandler{

    public:
    	//mavros_msgs::CommandBool arm_cmd;
    	ros::ServiceClient arming_client;
    	ros::ServiceClient set_mode_client;
    public:
    	StateHandler(ros::NodeHandle& node); //constructor
       ~StateHandler(); //destructor

    	void state_callb(const mavros_msgs::State::ConstPtr& msg);
    	ros::ServiceClient armModeClient(ros::NodeHandle& node);
    	ros::ServiceClient setModeClient(ros::NodeHandle& node);
		void Connect(ros::Rate& rate);
		void armDrone(mavros_msgs::CommandBool &arm_cmd,ros::ServiceClient& armingClient);
        void setOFFBOARD( mavros_msgs::SetMode& offb_set_mode, ros::ServiceClient& set_mode_client);

	private:
		ros::Subscriber state_sub;
        mavros_msgs::State state;
        
}; 

#endif