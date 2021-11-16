#include <ros/ros.h>

#include <stdio.h>
#include <unistd.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <string.h>
#include <errno.h>
#include <asm/ioctls.h>
#include <asm/termbits.h>

#define READ_TIMEOUT 10

int port;

int open_serial_port()
{
	int file_desc;
	file_desc = open("/dev/ydlidar", O_RDWR | O_NOCTTY | O_NONBLOCK | O_APPEND| O_NDELAY);

	if(file_desc == -1)
	{
		ROS_INFO("Port could not be opend.");
		return -1;
	}	

	struct termios2 config;

	// Baudrate
	config.c_cflag &= ~CBAUD;
	config.c_cflag |= BOTHER;

	config.c_ispeed = 128000;
	config.c_ospeed = 128000;

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
	{
		ROS_INFO("Port could not be opend.");
		return -1;
	}	

	if(ioctl(file_desc, TCSETS2, &config)==-1)
	{
		ROS_INFO("Port could not be opend.");
		return -1;
	}	

	return file_desc;
}

void read_initial_message()
{
	int counter = 0;
	char byte; 

	while(ros::ok() && counter < 4)
	{
	   	read(port, &byte, sizeof(byte));
		printf("%x\n", byte);


		if(byte == 0x0D)
			counter++;
	}
	ROS_INFO("Initial message read.");
}

int lidar_start_scan()
{
	uint8_t command_frame[] = { 0xA5, 0x60 };
	
	if(write(port, command_frame, sizeof(command_frame)) == -1)
	{
		ROS_INFO("Command was not sented.");
		return -1;
	}
	
	ROS_INFO("Command was sented.");
}

int lidar_stop_scan()
{
	uint8_t command_frame[] = { 0xA5, 0x65 };
	
	if(write(port, command_frame, sizeof(command_frame)) == -1)
	{
		ROS_INFO("Command was not sented.");
		return -1;
	}
	
	ROS_INFO("Command was sented.");
}

void lidar_read_data()
{
	ros::Rate rate(12);
	char byte[2]; 

	while(ros::ok())
	{
	   	read(port, &byte, sizeof(byte));
		printf("%x %x\n", byte[0], byte[1]);

		rate.sleep();
	}

	ROS_INFO("Stop reading.");
}

int main(int argc, char** argv)
{
   	ros::init(argc,argv,"lidar_node");
   	ros::NodeHandle nh;
	
	port = open_serial_port();
		
	if(port != -1)
	{
		read_initial_message();

		ros::Duration(2).sleep();

		lidar_start_scan();

		lidar_read_data();

		lidar_stop_scan();

	}

	

   	return 0;
}
