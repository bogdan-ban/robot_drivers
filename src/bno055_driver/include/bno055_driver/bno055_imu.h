#ifndef BNO055_IMU_H
#define BNO055_IMU_H

#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "drivers/uart_communication.h"
#include "drivers/i2c_communication.h"
#include "drivers/communication.h"
#include "drivers/sensor.h"
#include "drivers/CommunicationFactory.h"
#include "bno055_driver/acc.h"
#include "bno055_driver/mag.h"
#include "bno055_driver/gyr.h"
#include "bno055_driver/eul.h"
#include "bno055_driver/qua.h"
#include "bno055_driver/lia.h"
#include "bno055_driver/grv.h"
#include "bno055_driver/tmp.h"

#define ACC_START_BYTE 0x08
#define MAG_START_BYTE 0xE
#define GYR_START_BYTE 0x14
#define EUL_START_BYTE 0x1A
#define QUA_START_BYTE 0x20
#define LIA_START_BYTE 0x28
#define GRV_START_BYTE 0x2E

#define PUBLISH_TOPIC_ACC "/bno055/acc"
#define PUBLISH_TOPIC_MAG "/bno055/mag"
#define PUBLISH_TOPIC_GYR "/bno055/gyr"
#define PUBLISH_TOPIC_EUL "/bno055/eul"
#define PUBLISH_TOPIC_QUA "/bno055/qua"
#define PUBLISH_TOPIC_LIA "/bno055/lia"
#define PUBLISH_TOPIC_GRV "/bno055/grv"
#define PUBLISH_TOPIC_TMP "/bno055/tmp"

using namespace std;


std::vector<string> features = {"ACC","MAG","GYR","EUL","QUA","LIA","GRV","TEMP"};

class BNO055_IMU : public Sensor
{
private:
	ros::Publisher pub_acc;
	ros::Publisher pub_mag;
	ros::Publisher pub_gyr;
	ros::Publisher pub_eul;
	ros::Publisher pub_qua;
	ros::Publisher pub_lia;
	ros::Publisher pub_grv;
	ros::Publisher pub_tmp;

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
