#include "lidar_driver/lidar_driver.h"

int main(int argc, char** argv)
{
   	ros::init(argc,argv,"lidar_node");
   	ros::NodeHandle nh;

	Lidar lidar_sensor(&nh, "/dev/ydlidar");
	lidar_sensor.start_data_intake();

   	return 0;
}
