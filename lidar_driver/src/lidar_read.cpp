#include "lidar_driver/lidar_read.h"

Lidar_Read::Lidar_Read(ros::NodeHandle *nh, int baudrate, char* port_name): UartCommunication(port_name, baudrate)
{
	nh->getParam("raw_topic", topic_name);
	nh->getParam("frequency_read", frequency);

	printf("%d\n",frequency);

	pub = nh->advertise<std_msgs::UInt8>(topic_name, TOPIC_BUFFER_SIZE);
	pub_frame = nh->advertise<lidar_driver::frame>("/frame", TOPIC_BUFFER_SIZE);

	int port = get_file_desc();

	if(port != -1)
	{
		lidar_reset();
		ros::Duration(3).sleep();
		lidar_start_scan();
		ros::Duration(3).sleep();
		read_data(pub);
	}
	else
		ROS_INFO("Port was not open.");
}

Lidar_Read::~Lidar_Read()
{
	//stop();
}

void Lidar_Read::read_data(ros::Publisher pub)
{
	ROS_INFO("read: Reading data.");
	std_msgs::UInt8 byte_send;
	ros::Rate rate(frequency);

	uint8_t last_byte = 0 , counter = 0, byte = 0;
	bool frame_started= false;
	uint16_t no_bytes= 12;
	std::vector<uint8_t> buffer;

	while(ros::ok())
	{
		//byte_send.data = read_from_channel();
		//pub.publish(byte_send);
		//printf("%x\n",byte);

		byte = read_from_channel();
		//printf("%x\n",byte);
		std_msgs::UInt8 byte_s;
		byte_s.data = byte;
		pub.publish(byte_s);

		if(frame_started && counter <= no_bytes)
		{
			//ROS_INFO("Test for size.");
			switch(counter)
			{
				case 1:
				{
					//printf("byte: %x\n", byte);
					no_bytes += byte*2+1;
					//printf("no. bytes: %d\n", no_bytes);
					if(no_bytes <= 90)
						buffer.push_back(byte);
					else
						frame_started = false;
				} break;
				default: buffer.push_back(byte);
			}

			if(counter == no_bytes-1)
			{
				//for(int i=0;i<buffer.size();i++)
					//printf("%x ",buffer[i]);
				//buffer.clear();
				lidar_driver::frame frame;
				frame.frame_bytes = buffer;
				//std::cout << frame << std::endl;
				pub_frame.publish(frame);
				buffer.clear();

				//print_front();
				//ROS_INFO("Frame ended.");
				frame_started = false;
			}

			counter++;
			continue;
		}

		if((byte << 8 | last_byte) == 0x55aa)
		{
			//ROS_INFO("New Frame.");
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
