#include "lidar_driver/lidar_read.h"

Lidar_Read::Lidar_Read(ros::NodeHandle *nh, int baudrate, char* port_name): UartCommunication(port_name, baudrate)
{
	nh->getParam("raw_topic", topic_name);
	nh->getParam("frequency_read", frequency);

	pub_frame = nh->advertise<lidar_driver::frame>("/raw_frame", TOPIC_BUFFER_SIZE);

	int port = get_file_desc();

	if(port != -1)
	{
		lidar_reset();
		ros::Duration(3).sleep();
		lidar_start_scan();
		ros::Duration(3).sleep();
		ROS_INFO("Reading node: Reading data.");
		read_data();
	}
	else
		ROS_INFO("Port was not open.");
}

Lidar_Read::~Lidar_Read()
{
	//stop();
}

void Lidar_Read::read_data()
{
	std_msgs::UInt8 byte_send;
	ros::Rate rate(frequency);

	uint8_t last_byte = 0 , counter = 0, byte = 0;
	bool frame_started= false;
	uint16_t no_bytes= 12;
	std::vector<uint8_t> buffer;

	while(ros::ok())
	{
		byte = read_from_channel();

		if(frame_started && counter <= no_bytes)
		{
			switch(counter)
			{
				case 1:
				{
					no_bytes += byte*2+1;
					if(no_bytes <= 90)
						buffer.push_back(byte);
					else
						frame_started = false;
				} break;
				default: buffer.push_back(byte);
			}

			if(counter == no_bytes-1)
			{
				lidar_driver::frame frame;
				frame.frame_bytes = buffer;
				pub_frame.publish(frame);
				buffer.clear();

				frame_started = false;
			}

			counter++;
			continue;
		}

		if((byte << 8 | last_byte) == 0x55aa)
		{
			frame_started = true;
			counter = 0;
			no_bytes = 7;
			buffer.push_back(last_byte);
			buffer.push_back(byte);
		}
		else
			last_byte = byte;

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
