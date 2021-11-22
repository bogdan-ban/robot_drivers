#ifndef UART_COMMUNICATION_H
#define UART_COMMUNICATION_H

#include <iostream>
#include <string.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <errno.h>
#include <asm/ioctls.h>
#include <asm/termbits.h>

#include "robot_drivers/communication.h"

using namespace std;

class UartCommunication : public Communication
{
public:
	UartCommunication(char* port_name, int baud_rate);

	int start();

	uint8_t read_from_channel();

	void write_to_channel(const uint8_t buffer[],int size);

	void stop();

	int get_file_desc();

	// not inherited methods
	int convert_to_bytes(uint8_t vec[2]);

	void read_all_data(const string opt);

private:
	int file_desc;

	char* port;

	int baudrate;
};

#endif
