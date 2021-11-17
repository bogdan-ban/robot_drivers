#ifndef DATA_COLLECTOR_H
#define DATA_COLLECTOR_H

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


using namespace std;


class DataCollector
{
private:
	ros::Publisher pub;
	
public:

	// Communication* communication;

	UartCommunication* uc;

	I2cCommunication* i2c;

	DataCollector(ros::NodeHandle *nh);

};

#endif
