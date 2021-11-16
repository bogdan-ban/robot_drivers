#include "lidar_driver/lidar_read.h"

int main(int argc, char** argv)
{
   	ros::init(argc,argv,"read_node");
   	ros::NodeHandle nh;

	std::string port_name;
	int baudrate;

	nh.getParam("port_name",port_name);
	nh.getParam("baudrate", baudrate);

	Lidar_Read rd(&nh, baudrate, (char*)port_name.c_str());

   	return 0;
}
