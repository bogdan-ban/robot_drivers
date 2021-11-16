#include "lidar_driver/lidar_driver.h"

Lidar::Lidar(ros::NodeHandle *nh, std::string _communication_channer): Sensor(_communication_channer)
{
	ROS_INFO("Lidar sensor initialized.");

	pub = nh->advertise<sensor_msgs::LaserScan>(topic_name, PUBLISHER_QUE_SIZE);
	pub_frame = nh->advertise<lidar_driver::frame>("/frame", PUBLISHER_QUE_SIZE);

	port = open_serial_port();

	if(port != -1)
		ROS_INFO("Port opened.");
	else
		ROS_INFO("Port NOT opened.");

}

Lidar::~Lidar()
{
	if(port != -1)
	{
		ROS_INFO("Closing port.");
		close(port);
		lidar_stop_scan();
	}
	else
		ROS_INFO("Error occure at port opening.");
}

int Lidar::open_serial_port()
{
	int file_desc;
	file_desc = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_APPEND| O_NDELAY);

	if(file_desc == -1)
		return -1;

	struct termios2 config;

	// Baudrate
	config.c_cflag &= ~CBAUD;
	config.c_cflag |= BOTHER;

	config.c_ispeed = baudrate;
	config.c_ospeed = baudrate;

	config.c_cflag &= ~PARENB;   		// No parity bite
	config.c_cflag &= ~CSTOPB;   		// 1 bit stop bit

	// Set 8 bit data bit	
	config.c_cflag &= ~CSIZE;
	config.c_cflag |= CS8;

	config.c_cflag &= ~CRTSCTS;			// No hardware flow control
	config.c_cflag |= CREAD | CLOCAL;	// Turn on READ and ignore ctrl lines

	// Return immediately what is available
    config.c_cc[VTIME] = 0;
    config.c_cc[VMIN] = 0;
	
	// No s/w control
	config.c_iflag &= ~(IXON | IXOFF | IXANY);
	config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
	
	// Canonical input off
	config.c_lflag &= ~ICANON;

	// No echo
	config.c_lflag &= ~ECHOE;
	config.c_lflag &= ~ECHONL;
	config.c_lflag &= ~ISIG;

	if(fcntl(file_desc, F_SETFL, FNDELAY))
		return -1;

	if(ioctl(file_desc, TCSETS2, &config)==-1)
		return -1;	

	return file_desc;
}

int Lidar::lidar_start_scan()
{
	uint8_t command_frame[] = { COMMAND_START_BYTE, COMMAND_START_SCAN_BYTE };
	
	if(write(port, command_frame, sizeof(command_frame)) == -1)
	{
		ROS_INFO("Start Command was not sented.");
		return -1;
	}
	
	ROS_INFO("Start command was sented.");
	return 1;
}

int Lidar::lidar_reset()
{
	uint8_t command_frame[] = { COMMAND_START_BYTE, COMMAND_RESET_BYTE };
	
	if(write(port, command_frame, sizeof(command_frame)) == -1)
	{
		ROS_INFO("Reset command was not sented.");
		return -1;
	}
	
	ROS_INFO("Reset command was sented.");
	return 1;
}

int Lidar::lidar_health()
{
	uint8_t command_frame[] = { COMMAND_START_BYTE, COMMAND_HEALTH_INFO_BYTE };
	
	if(write(port, command_frame, sizeof(command_frame)) == -1)
	{
		ROS_INFO("Reset command was not sented.");
		return -1;
	}
	
	ROS_INFO("Reset command was sented.");
	return 1;
}

int Lidar::lidar_stop_scan()
{
	uint8_t command_frame[] = { COMMAND_START_BYTE, COMMAND_STOP_SCAN_BYTE };
	
	if(write(port, command_frame, sizeof(command_frame)) == -1)
	{
		ROS_INFO("Stop command was not sented.");
		return -1;
	}
	
	ROS_INFO("Stop command was sented.");
	return 1;
}

void Lidar::read_initial_message()
{
	ROS_INFO("Reading initial message.");
	uint8_t initial_message_buffer[250];

	int bytes_read = -2;
	bytes_read = read(port, initial_message_buffer, sizeof(initial_message_buffer));
	printf("%d\n",bytes_read);

	for(int i = 0; i<bytes_read; i++)
		printf("%x\n", initial_message_buffer[i]);

	printf("\n");

	if(bytes_read > 0)
	{
		if(initial_message_buffer[bytes_read - 1] == CARRIAGE_RETURN_BYTE && bytes_read >= MAX_INITIAL_MESSAGE_LENGTH - 2)
			ROS_INFO("Initial message read.");
	}
	else
		if(bytes_read <= 0)
			ROS_INFO("The message was not read. Some bytes may hav been lost"); 
}

void Lidar::lidar_find_head_frame()
{
	//ros::Rate rate(frequency);
	//uint8_t byte_LSB = 0, byte_MSB = 0;
	uint8_t  initial_reply[7];
	//uint16_t byte; 

	uint8_t bytes_read = -2;
	bytes_read = read(port, initial_reply, sizeof(initial_reply));

	//printf("%d\n",bytes_read);
	if(bytes_read > 0)
	{
		//for(int i=0;i<bytes_read;i++)	
			//printf("%x\n",initial_reply[i]);
		if(initial_reply[bytes_read - 1] == 0x81 && bytes_read >= MAX_INITIAL_MESSAGE_LENGTH - 2)
			ROS_INFO("Reply message read.");
	}
	else
		if(bytes_read <= 0)
			ROS_INFO("Reply message NOT read."); 

	// Find the head of the initial reply
	/*while(ros::ok() && byte != FRAME_HEAD)
	{
		read(port, &byte_LSB, sizeof(byte_LSB));
		byte = byte_MSB << 8 | byte_LSB;
		byte_MSB = byte_LSB;
		byte_LSB = 0;
		rate.sleep();
	}	*/
	//ROS_INFO("Head found.");
}

float Lidar::angle_to_rad(float angle)
{
	return (angle*3.1415)/180;
}

void Lidar::initial_scan_setup(sensor_msgs::LaserScan *laser_scan)
{
	//laser_scan->header.seq = 0;
	laser_scan->header.stamp = ros::Time::now();
	laser_scan->header.frame_id = frame_id;

	laser_scan->angle_min = angle_to_rad(min_angle);
	laser_scan->angle_max = angle_to_rad(max_angle);

	laser_scan->range_min = min_distance;
	laser_scan->range_max = max_distance;
}

bool Lidar::process_frame(std::vector<uint8_t> buffer)
{		
	uint16_t diatance_byte, start_angle_byte, stop_angle_byte;
	uint8_t no_sample_points;	
	float start_angle, end_angle, diff_angle, angle_i, distance, angle_cor;
	int index = 0, start_index, end_index, counter = 2;
	
	//for(int i=0;i<buffer.front().size();i++)
		//printf("%x ",buffer.front()[i]);	
	//printf("\n\n");	

	if(buffer[2] != 1)
	{
		no_sample_points = buffer[3];	
		
		start_angle_byte = buffer[5] << 8 | buffer[4];
		stop_angle_byte = buffer[7] << 8 | buffer[6];

		start_angle = (float)(start_angle_byte >> 1) / 64;
		end_angle = (float)(stop_angle_byte >> 1) / 64;

		if(start_angle > end_angle)
			diff_angle = 360 - start_angle + end_angle;
		else
			diff_angle = end_angle - start_angle;

		//printf("%d %.4f %.4f\n", no_sample_points, start_angle, end_angle);	
	
		start_index = start_angle / (diff_angle / no_sample_points);
		end_index = end_angle / (diff_angle / no_sample_points);
		
		if(start_stop_index == -1)
			start_stop_index = start_index;

		printf("%d %d %d\n",start_index, end_index, start_stop_index);

		for(int i=1; i<no_sample_points*2-2; i+=2)
		{
			angle_i = (diff_angle / (no_sample_points - 1)) * (counter - 1) + start_angle;
			
			diatance_byte = buffer[10+i] << 8 | buffer[10+i-1];
			distance = ((float)diatance_byte) / 4 / 1000;

			if(distance == 0)
				angle_cor = 0;
			else
				angle_cor = atan(21.8*((155.3-distance*1000)/(155.3+distance*1000)));
			angle_i = angle_i + angle_cor;
		
			angle_i = 720 - angle_i;		
	
			if(angle_i > 360)
				angle_i -= 360;

			index = angle_i / (diff_angle / no_sample_points);

			if(distance < 0 || angle_i < min_angle || angle_i > max_angle || distance < min_distance || distance > max_distance)
			{
				distance = 0;
				angle_i = 0;		
			}
			
			distance_array[index] = distance;

			counter++;

			//printf("%d %.4f %.4f\n", index, angle_i, distance);
		}

		//if(abs(start_stop_index - start_index) < FULL_SCAN_TOLERANCE)
		if(start_stop_index > start_index && start_stop_index <= end_index)		
		{
			ROS_INFO("PUBLISH");
			//printf("%d\n", start_stop_index);
			sensor_msgs::LaserScan laser_scan;

			initial_scan_setup(&laser_scan);

			laser_scan.header.stamp = ros::Time::now();
			laser_scan.angle_increment = angle_to_rad(0.625);
			laser_scan.time_increment = 0.0002;
			laser_scan.scan_time = 0.112;
			laser_scan.ranges = std::vector<float>(distance_array, distance_array + sizeof(distance_array)/sizeof(distance_array[0]));
			laser_scan.header.seq = seq;
			seq++;

			//std::cout << laser_scan << std::endl;
			pub.publish(laser_scan);
			//ros::Duration(0.002).sleep();

			for(int i=0;i<600;i++)
				distance_array[i] = 0;
	
			//tcflush(port, TCIFLUSH);
			//ros::Duration(1).sleep();
			//ros::Duration(0.1).sleep();
				
			//lidar_health();

			if(ioctl(port, TCFLSH, 0) < 0)
				ROS_INFO("error at reseting");

			//buffer.pop();
			//return true;

			/*close(port);
			port = -1;
			ros::Duration(3).sleep();
			port = open_serial_port();*/
			

			/*ros::Duration(2).sleep();
			ROS_INFO("resetting lidar");
			lidar_reset();
			ROS_INFO("lidar reset");
			ros::Duration(2).sleep();
			start_stop_index = -1;*/

			//read_initial_message();
			//ros::Duration(1).sleep();*/	
			
			//ROS_INFO("started scanning");
			//ros::Duration(1).sleep();	
			//ros::spinOnce();
			
			//lidar_stop_scan();
			//lidar_start_scan();
		}
	}
	
	//print_buffer();
	
	return false;
}

void Lidar::print_buffer()
{
	ROS_INFO("Printing Buffer");
	while(buffer.size() > 0)	
	{
		for(int i=0;i<buffer.front().size(); i++)
			printf("%x ", buffer.front()[i]);
		printf("\n");
		buffer.pop();
	}
}

void Lidar::lidar_read_data()
{
	ROS_INFO("Start measurements.");
	ros::Rate rate(frequency);

	std::vector<uint8_t> v_buffer;
	uint8_t byte = 0, last_byte = 0, counter = 0;
	bool frame_started = false, frame_processed = false;
	uint16_t no_bytes = 12, error;

	while(ros::ok() && port != -1)
	{
		error = read(port, &byte, sizeof(byte));
		//printf("%x\n",byte);		

		if(error > 0)
		{
			if(frame_started && counter <= no_bytes)
			{
				//printf("%x\n",byte);
				switch(counter)
				{
					case 1: 
					{
						no_bytes += byte*2+1; 
						buffer.back().push_back(byte);
					} break;
					default: buffer.back().push_back(byte);
				}

				if(counter == no_bytes)
				{
					ROS_INFO("Frame ended.");
					frame_started = false;
					//if(ioctl(port, TCFLSH, 0) < 0)
						//ROS_INFO("error at reseting");
				}

				counter++;
				continue;
			}
		
			if(buffer.size() > 1)
			{
				lidar_driver::frame frame;
				frame.frame_bytes = buffer.front();
				pub_frame.publish(frame);
				frame_processed = process_frame(buffer.front());
				ROS_INFO("Pop.");
				buffer.pop();
			}

			if((byte << 8 | last_byte) == 0x55aa)
			{
				ROS_INFO("New Frame.");
				frame_started = true;
				counter = 0;
				no_bytes = 7;
				buffer.push(std::vector<uint8_t>());
				buffer.back().push_back(last_byte);
				buffer.back().push_back(byte);
				printf("%d\n",buffer.size());
				continue;
			}
			else
				last_byte = byte;
		}
		else
		{	
			ROS_INFO("error at reading");
		}

		rate.sleep();
	}
	//print_buffer();

	ROS_INFO("Stop reading.");
}

void Lidar::start_data_intake()
{
	if(port != -1)
	{
		//ros::Duration(WAIT_SECONDS_INITIAL_MSG).sleep();
	
		//read_initial_message();

		ros::Duration(WAIT_SECONDS_INITIAL_MSG).sleep();

		lidar_start_scan();

		ros::Duration(0.1).sleep();
		
		lidar_read_data();
	}
	else
		ROS_INFO("Port could not be open.");
}

void Lidar::change_path(std::string new_comunication_channel) 
{
	communication_channel = new_comunication_channel;
}

