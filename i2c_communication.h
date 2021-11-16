#ifndef I2C_COMMUNICATION_H
#define I2C_COMMUNICATION_H

#include <ros/ros.h>
#include <iostream>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <fcntl.h>
#include "communication.h"

using namespace std;

class I2cCommunication : public Communication
{
public:

	I2cCommunication(char* port_name, int addr);

	int start();

	uint8_t read_from_channel();

	void write_to_channel(const uint8_t buffer[],int size);

	void stop();

	int convert_to_bytes(uint8_t vec[2]);

	void read_all_data(const string opt);

private:

	char* port;

	int fd;

	int address;

};

#endif