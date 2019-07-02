#include "../include/turtlebot_rrtstar/RRTStar.h"
#include "../include/turtlebot_rrtstar/Controller.h"

// int main()
int main(int argc, char** argv)
{


    ros::init(argc, argv, "turtlebot_Controller"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    Controller controller(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("Initializing...");
    ros::spin();
	return 0;
}