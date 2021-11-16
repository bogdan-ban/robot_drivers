#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <unistd.h>
#include <string.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <errno.h>
#include <asm/ioctls.h>
#include <asm/termbits.h>

#include "uart_communication.h"

#define COMMAND_START_BYTE 0xA5
#define COMMAND_START_SCAN_BYTE 0x60
#define COMMAND_STOP_SCAN_BYTE 0x65
#define COMMAND_RESET_BYTE 0x80

#define TOPIC_BUFFER_SIZE 100

class Lidar_Read: public UartCommunication
{
private:
	int port;

	std::string topic_name = "/bytes";
	std::string port_name = "/dev/ydlidar";
	int baudrate = 128000;
	int frequency = 4500;

	ros::Publisher pub;

public:
	Lidar_Read(ros::NodeHandle *nh, int baudrate, char* port_name);
	~Lidar_Read();

	void read_data(ros::Publisher pub);
	int lidar_start_scan();
	int lidar_stop_scan();
	int lidar_reset();
};
