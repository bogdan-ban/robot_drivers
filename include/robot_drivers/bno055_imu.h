#ifndef BNO055_IMU_H
#define BNO055_IMU_H

#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
// #include <termios.h>
#include <string.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "uart_communication.h"
#include "i2c_communication.h"
#include "communication.h"
#include "sensor.h"
#include "CommunicationFactory.h"


using namespace std;


class BNO055_IMU : public Sensor
{
private:
	ros::Publisher pub;

	string option;

public:

	BNO055_IMU(ros::NodeHandle* nh, Communication* c);

	int convert_to_bytes(uint8_t vec[2]);

	void read_all_data_UART(const string opt);

	void read_all_data_I2C(const string opt);

	void set_option(const string _opt);

	Communication* get_communication();

	// inherited functions
	void start_communication();

	void stop_communication();

	void read_data(uint8_t* buffer, int size);
	
	void send_command(uint8_t* command);

	void publish_message();

	void create_message();

};

#endif
