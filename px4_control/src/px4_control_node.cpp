
#include "px4_control.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "CNN_Autonomous_Navigation");
    ros::NodeHandle nh;
    px4_control cnn_publisher(nh);
    ros::spin();

    
  
    return 0;
}