#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/LaserScan.h>
#include <lidar_driver/frame.h>

#include <unistd.h>
#include <string.h>

#include <queue>
#include <vector>

#include "driver_config.h"

class Lidar_Process
{
private:
	// Default valuse
	uint8_t last_byte = 0, counter = 0;
	bool frame_started = false;
	uint16_t no_bytes = 12;

	float distance_array[MAX_NUMBER_DISTANCES] = {0};

	int seq = 0;

	float max_distance = 8.0f;
	float min_distance = 0.2f; 

	float max_angle = 360;
	float min_angle = 0;

	std::string frame_id = "laser_frame";
	std::string read_topic_name = "/raw_frame";
	std::string scan_topic_name = "/scan_frame";

	ros::Publisher pub;
	ros::Subscriber sub;

	sensor_msgs::LaserScan laser_scan;

	double start_time_scan = 0;
	double start_time_measure = 0;
public:
	Lidar_Process(ros::NodeHandle *nh);

	float angle_to_rad(float angle);
	void initial_scan_setup(sensor_msgs::LaserScan *laser_scan);
	void process_frame(std::vector<uint8_t> buffer);
	void recevied_bytes(const lidar_driver::frame msg);
};
