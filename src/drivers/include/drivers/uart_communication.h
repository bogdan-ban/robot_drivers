#ifndef UART_COMMUNICATION_H
#define UART_COMMUNICATION_H

#include <iostream>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <errno.h>
#include <asm/ioctls.h>
#include <vector>

#include "communication.h"

#define termios asmtermios
#include <asm/termbits.h>
#undef termios
#include <termios.h>

using namespace std;

class UartCommunication : public Communication
{
public:
	UartCommunication(char* port_name);

	UartCommunication(char* port_name, int baud_rate);

	int start();

	int start_bno055();

	uint8_t read_from_channel();

	void write_to_channel(const uint8_t buffer[],int size);

	void stop();

	int get_file_desc();

	void flush();

private:
	int file_desc;

	char* port;

	int baudrate;
};

#endif
