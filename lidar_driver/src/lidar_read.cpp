#include "lidar_driver/lidar_read.h"

Lidar_Read::Lidar_Read(ros::NodeHandle *nh, int baudrate, char* port_name): UartCommunication(port_name, baudrate)
{
	nh->getParam("raw_topic", topic_name);
	nh->getParam("frequency_read", frequency);

	printf("%d\n",frequency);

	ros::Publisher pub = nh->advertise<std_msgs::UInt8>(topic_name, TOPIC_BUFFER_SIZE);

	int port = get_file_desc();

	if(port != -1)
	{
		lidar_reset();
		ros::Duration(5).sleep();
		lidar_start_scan();
		ros::Duration(3).sleep();
		read_data(pub);
	}
	else
		ROS_INFO("Port was not open.");
}

Lidar_Read::~Lidar_Read()
{
	stop();
}

void Lidar_Read::read_data(ros::Publisher pub)
{
	ROS_INFO("read: Reading data.");
	std_msgs::UInt8 byte_send;
	ros::Rate rate(frequency);
	while(ros::ok())
	{
		byte_send.data = read_from_channel();
		pub.publish(byte_send);
		//printf("%x\n",byte);
		rate.sleep();
	}
}

int Lidar_Read::lidar_start_scan()
{
	uint8_t command_frame[] = { COMMAND_START_BYTE, COMMAND_START_SCAN_BYTE };

	write_to_channel(command_frame, sizeof(command_frame));

	return 1;
}

int Lidar_Read::lidar_stop_scan()
{
	uint8_t command_frame[] = { COMMAND_START_BYTE, COMMAND_STOP_SCAN_BYTE };

	write_to_channel(command_frame, sizeof(command_frame));

	return 1;
}

int Lidar_Read::lidar_reset()
{
	uint8_t command_frame[] = { COMMAND_START_BYTE, COMMAND_RESET_BYTE };

	write_to_channel(command_frame, sizeof(command_frame));

	return 1;
}
