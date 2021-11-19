#include "lidar_driver/lidar_process.h"

int main(int argc, char** argv)
{
   	ros::init(argc,argv,"process_node");
   	ros::NodeHandle nh;

	Lidar_Process lidar_process(&nh);

	ROS_INFO("Processing node: Processing node ready.");

	ros::spin();

   	return 0;
}
