#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <lidar_driver/frame.h>
#include <unistd.h>
#include <string.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <errno.h>
#include <asm/ioctls.h>
#include <asm/termbits.h>

#include "lidar_driver/sensor.h"
#include "lidar_driver/communication.h"

#include "lidar_driver/driver_config.h"

class Lidar_Read: public Sensor
{
private:
	int port;

	// Default values
	std::string topic_name = "/raw_frame";
	std::string port_name = "/dev/ydlidar";
	int baudrate = 128000;
	int frequency = 5000;

	ros::Publisher pub_frame;

	std::vector<uint8_t> buffer;
	lidar_driver::frame frame;

public:
	Lidar_Read(ros::NodeHandle *nh, Communication *comm);
	~Lidar_Read();

	void process_bytes();
	int lidar_start_scan();
	int lidar_stop_scan();
	int lidar_reset();

	void start_communication();
	void stop_communication();
	void read_data(uint8_t* buffer,int size);
	void send_command(uint8_t* command);
	void create_message();
	void publish_message();
};
