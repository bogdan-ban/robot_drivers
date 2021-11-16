#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <lidar_driver/frame.h>

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <cmath>
#include <math.h>
#include <algorithm>
#include <string.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <errno.h>
#include <asm/ioctls.h>
#include <asm/termbits.h>

#include <queue>
#include <vector>

#include "lidar_driver/Sensor.h"

#define COMMAND_START_BYTE 0xA5
#define COMMAND_START_SCAN_BYTE 0x60
#define COMMAND_STOP_SCAN_BYTE 0x65
#define COMMAND_HEALTH_INFO_BYTE 0x91
#define COMMAND_RESET_BYTE 0x80

#define FRAME_HEAD 0xA55A
#define CONTEND_HEAD 0xAA55

#define NEW_LINE_BYTE 0xA8
#define CARRIAGE_RETURN_BYTE 0x0D
#define INITIAL_MESSAGE_NO_LINE 4
#define MAX_INITIAL_MESSAGE_LENGTH 182

#define COMMAND_START_STOP_MAX_LENGTH 4
#define COMMAND_RESPONSE_MAX_LENGTH 2

#define WAIT_SECONDS_INITIAL_MSG 3
#define MAX_MEASURED_POINTS 50
#define ANGLE_INCREMENT 0.625
#define FULL_SCAN_TOLERANCE 20

#define PUBLISHER_QUE_SIZE 15

#define READ_TIMEOUT 10  //miliseconds

class Lidar: public Sensor
{
protected:
	// TO DO get params from file
	float frequency = 10.0f;

	std::string port_name = "/dev/ydlidar"; // or "/dev/ttyUSB0"
	int baudrate = 128000;

	std::string frame_id = "lase_frame";
	std::string topic_name = "/scan_laser";

	float max_distance = 8.0f;
	float min_distance = 0.1f; 

	float max_angle = 360;
	float min_angle = 0;

	ros::Publisher pub;
	ros::Publisher pub_frame;

	int seq = 0;

	int port;
	bool is_scanning = false; 

	bool laser_scan_initial = false;

	std::queue<std::vector<uint8_t>> buffer;

	float distance_array[600] = {0};

	int start_stop_index = -1;

private:
	int open_serial_port();
	void read_initial_message();

	int lidar_start_scan();
	int lidar_reset();

	bool process_frame(std::vector<uint8_t> buffer);
	void initial_scan_setup(sensor_msgs::LaserScan *laser_scan);

	int lidar_stop_scan();
	int lidar_health();

	void lidar_find_head_frame();
	void lidar_read_data();

	float angle_to_rad(float angle);

	void print_buffer();
public:
	Lidar(ros::NodeHandle *nh, std::string _communication_channer);

	void start_data_intake();

	void change_path(std::string new_comunication_channel);    

	~Lidar();
};


