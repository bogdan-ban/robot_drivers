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
#include "drivers/uart_communication.h"
#include "drivers/i2c_communication.h"
#include "drivers/communication.h"
#include "drivers/sensor.h"
#include "drivers/CommunicationFactory.h"

#define ACC_START_BYTE 0x08
#define MAG_START_BYTE 0xE
#define GYR_START_BYTE 0x14
#define EUL_START_BYTE 0x1A
#define QUA_START_BYTE 0x20
#define LIA_START_BYTE 0x28
#define GRV_START_BYTE 0x2E

using namespace std;


std::vector<string> features = {"ACC","MAG","GYR","EUL","QUA","LIA","GRV"};

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
