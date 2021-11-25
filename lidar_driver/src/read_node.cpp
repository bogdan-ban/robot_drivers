#include "lidar_driver/lidar_read.h"
#include "lidar_driver/CommunicationFactory.h"

int main(int argc, char** argv)
{
   	ros::init(argc,argv,"read_node");
   	ros::NodeHandle nh;

	std::string port_name;
	int baudrate;

	nh.getParam("port_name",port_name);
	nh.getParam("baudrate", baudrate);

	CommunicationFactory cf = CommunicationFactory();

	Lidar_Read rd(&nh, cf.create_communication("UART", (char *)port_name.c_str(), baudrate));

   	return 0;
}
