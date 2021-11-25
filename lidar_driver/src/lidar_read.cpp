#include "lidar_driver/lidar_read.h"

Lidar_Read::Lidar_Read(ros::NodeHandle *nh, Communication *comm): Sensor(comm)
{
	nh->getParam("raw_topic", topic_name);
	nh->getParam("frequency_read", frequency);

	pub_frame = nh->advertise<lidar_driver::frame>("/raw_frame", TOPIC_BUFFER_SIZE);

	if(port != -1)
	{
		lidar_reset();
		ros::Duration(3).sleep();
		lidar_start_scan();
		ros::Duration(3).sleep();
		ROS_INFO("Reading node: Reading data.");
		process_bytes();
	}
	else
		ROS_INFO("Port was not open.");
}

Lidar_Read::~Lidar_Read()
{
	//stop();
}

void Lidar_Read::process_bytes()
{
	std_msgs::UInt8 byte_send;
	ros::Rate rate(frequency);

	uint8_t last_byte = 0 , counter = 0, byte = 0;
	bool frame_started= false;
	uint16_t no_bytes= 12;

	while(ros::ok())
	{
		read_data(&byte,0);

		if(frame_started && counter <= no_bytes)
		{
			switch(counter)
			{
				case 1:
				{
					no_bytes += byte*2+1;
					if(no_bytes <= MAX_FRAME_LENGTH)
						buffer.push_back(byte);
					else
						frame_started = false;
				} break;
				default: buffer.push_back(byte);
			}

			if(counter == no_bytes-1)
			{
				create_message();
				publish_message();
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

	send_command(command_frame);

	return 1;
}

int Lidar_Read::lidar_stop_scan()
{
	uint8_t command_frame[] = { COMMAND_START_BYTE, COMMAND_STOP_SCAN_BYTE };

	send_command(command_frame);

	return 1;
}

int Lidar_Read::lidar_reset()
{
	uint8_t command_frame[] = { COMMAND_START_BYTE, COMMAND_RESET_BYTE };

	send_command(command_frame);

	return 1;
}

void Lidar_Read::start_communication()
{
	//port = communication->get_file_desc();
}

void Lidar_Read::stop_communication()
{
	communication->stop();
}

void Lidar_Read::read_data(uint8_t* buffer,int size)
{
	*buffer = communication->read_from_channel();
}

void Lidar_Read::send_command(uint8_t* command)
{
	communication->write_to_channel(command, sizeof(command));
}

void Lidar_Read::publish_message()
{
	pub_frame.publish(*static_cast<lidar_driver::frame*>(message));
}

void Lidar_Read::create_message()
{
	frame.frame_bytes = buffer;
	message = &frame;
}
