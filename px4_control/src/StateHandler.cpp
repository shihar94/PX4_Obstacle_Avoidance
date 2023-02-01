#include "StateHandler.h"

//constructor
StateHandler::StateHandler(ros::NodeHandle& node)
{
	state_sub = node.subscribe<mavros_msgs::State>("mavros/state",10, &StateHandler::state_callb,this);

}

/*ros::ServiceClient StateHandler::armModeClient(ros::NodeHandle& node){
    	arming_client=node.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming",this);
    	return arming_client;
}*/

/*ros::ServiceClient StateHandler::setModeClient(ros::NodeHandle& node){
 	 set_mode_client=node.serviceClient<mavros_msgs::SetMode>("mavros/set_mode",this);
 	 return set_mode_client;
}*/

//the callback function
void StateHandler::state_callb(const mavros_msgs::State::ConstPtr& msg){
	state = *msg;
}
 
//the destructor
StateHandler::~StateHandler()
{
	ROS_INFO("CLOSING DOWN");
}

void StateHandler::armDrone(mavros_msgs::CommandBool &arm_cmd,ros::ServiceClient& armingClient){
	if(!state.armed){
		///ROS_INFO(".....Arming The Vehicle......");
		arm_cmd.request.value = true;
		//ros::ServiceClient armingClient=node.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
		if( armingClient.call(arm_cmd) && arm_cmd.response.success){
      		ROS_INFO("Vehicle armed");
    	}
	}
	//ROS_INFO("Vehicle armed");
}

void StateHandler::setOFFBOARD( mavros_msgs::SetMode& offb_set_mode, ros::ServiceClient& set_mode_client){
	//ros::ServiceClient set_mode_client=node.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	if(state.mode !="OFFBOARD"){
		if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
			ROS_INFO("OFFBOARD ENABLED");
		}else{
			ROS_INFO("OFFBOARD not ENABLED");
		}
    
   
	}
	//ROS_INFO("OFFBOARD ENABLED");
}




void StateHandler::Connect(ros::Rate& rate){
	ROS_INFO("Establishing FCU Connection");
	while(ros::ok() && !state.connected){
		
		ros::spinOnce();
		rate.sleep();
	}
    
}
    




