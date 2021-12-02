#include "drivers/uart_communication.h"

UartCommunication::UartCommunication(char* port_name, int baud_rate): port(port_name), baudrate(baud_rate)
{
	start();
}

int UartCommunication::start()
{
	file_desc = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK | O_APPEND| O_NDELAY);

	if(file_desc == -1)
		return -1;

	struct termios2 config;

	// Baudrate
	config.c_cflag &= ~CBAUD;
	config.c_cflag |= BOTHER;

	config.c_ispeed = baudrate;
	config.c_ospeed = baudrate;

	config.c_cflag &= ~PARENB;   		// No parity bite
	config.c_cflag &= ~CSTOPB;   		// 1 bit stop bit

	// Set 8 bit data bit
	config.c_cflag &= ~CSIZE;
	config.c_cflag |= CS8;

	config.c_cflag &= ~CRTSCTS;			// No hardware flow control
	config.c_cflag |= CREAD | CLOCAL;	// Turn on READ and ignore ctrl lines

	// Return immediately what is available
    	config.c_cc[VTIME] = 0;
    	config.c_cc[VMIN] = 0;

	// No s/w control
	config.c_iflag &= ~(IXON | IXOFF | IXANY);
	config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

	// Canonical input off
	config.c_lflag &= ~ICANON;

	// No echo
	config.c_lflag &= ~ECHOE;
	config.c_lflag &= ~ECHONL;
	config.c_lflag &= ~ISIG;

	if(fcntl(file_desc, F_SETFL, FNDELAY))
		return -1;

	if(ioctl(file_desc, TCSETS2, &config)==-1)
		return -1;

	return file_desc;
}

uint8_t UartCommunication::read_from_channel()
{
	uint8_t byte, error;
	error = read(file_desc, &byte, sizeof(byte));
	if(error >= 1)
		return byte;

	return -1;
}

void UartCommunication::write_to_channel(const uint8_t buffer[],int size)
{
	if(buffer != nullptr && size > 0)
		write(file_desc, buffer, size);
}

void UartCommunication::stop()
{
	if(file_desc!=-1)
		close(file_desc);
}

int UartCommunication::get_file_desc()
{
	return file_desc;
}
