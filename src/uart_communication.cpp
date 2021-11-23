#include "robot_drivers/uart_communication.h"

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

int UartCommunication::convert_to_bytes(uint8_t vec[2])
{
	return 256 * vec[0] + vec[1];
}

void UartCommunication::read_all_data(const string opt)
{
	uint8_t read_DATA_X_MSB[4] = { 0xAA, 0x01, 0x0, 0x01};
	uint8_t read_DATA_X_LSB[4] = { 0xAA, 0x01, 0x0, 0x01};
	uint8_t read_DATA_Y_MSB[4] = { 0xAA, 0x01, 0x0, 0x01};
	uint8_t read_DATA_Y_LSB[4] = { 0xAA, 0x01, 0x0, 0x01};
	uint8_t read_DATA_Z_MSB[4] = { 0xAA, 0x01, 0x0, 0x01};
	uint8_t read_DATA_Z_LSB[4] = { 0xAA, 0x01, 0x0, 0x01};

	// for quaternion
	uint8_t read_DATA_W_MSB[4] = { 0xAA, 0x01, 0x21, 0x01};
	uint8_t read_DATA_W_LSB[4] = { 0xAA, 0x01, 0x20, 0x01};

	if(opt == "ACC")
	{
		read_DATA_X_MSB[2] = '\x09';
		read_DATA_X_LSB[2] = '\x08';
		read_DATA_Y_MSB[2] = '\x0B';
		read_DATA_Y_LSB[2] = '\x0A';
		read_DATA_Z_MSB[2] = '\x0D';
		read_DATA_Z_LSB[2] = '\x0C';
		printf("ACC: \n");
	}
	else if(opt == "MAG")
	{
		read_DATA_X_MSB[2] = '\x0F';
		read_DATA_X_LSB[2] = '\x0E';
		read_DATA_Y_MSB[2] = '\x11';
		read_DATA_Y_LSB[2] = '\x10';
		read_DATA_Z_MSB[2] = '\x13';
		read_DATA_Z_LSB[2] = '\x12';
		printf("MAG: \n");
	}
	else if(opt == "GYR")
	{
		read_DATA_X_MSB[2] = '\x15';
		read_DATA_X_LSB[2] = '\x14';
		read_DATA_Y_MSB[2] = '\x17';
		read_DATA_Y_LSB[2] = '\x16';
		read_DATA_Z_MSB[2] = '\x19';
		read_DATA_Z_LSB[2] = '\x18';
		printf("GYR: \n");
	}
	else if(opt == "EUL")
	{
		read_DATA_X_MSB[2] = '\x1B';
		read_DATA_X_LSB[2] = '\x1A';
		read_DATA_Y_MSB[2] = '\x1D';
		read_DATA_Y_LSB[2] = '\x1C';
		read_DATA_Z_MSB[2] = '\x1F';
		read_DATA_Z_LSB[2] = '\x1E';
		printf("EUL: \n");
	}
	else if(opt == "QUA")
	{
		read_DATA_X_MSB[2] = '\x23';
		read_DATA_X_LSB[2] = '\x22';
		read_DATA_Y_MSB[2] = '\x25';
		read_DATA_Y_LSB[2] = '\x24';
		read_DATA_Z_MSB[2] = '\x27';
		read_DATA_Z_LSB[2] = '\x26';
		printf("QUA: \n");
		// w
		uint8_t vec[2];
		write_to_channel(read_DATA_W_MSB,4);
		vec[1] = read_from_channel();
		//printf("LSB: %x\n", vec[1]);
		//ros::Duration(0.1).sleep();

		write_to_channel(read_DATA_W_LSB,4);
		vec[0] = read_from_channel();
		//printf("MSB: %x\n", vec[0]);
		//ros::Duration(0.1).sleep();

		printf("w: %d\n",convert_to_bytes(vec));
	}
	else if(opt == "LIA")
	{
		read_DATA_X_MSB[2] = '\x29';
		read_DATA_X_LSB[2] = '\x28';
		read_DATA_Y_MSB[2] = '\x2B';
		read_DATA_Y_LSB[2] = '\x2A';
		read_DATA_Z_MSB[2] = '\x2D';
		read_DATA_Z_LSB[2] = '\x2C';
		printf("LIA: \n");
	}
	else if(opt == "GRV")
	{
		read_DATA_X_MSB[2] = '\x2F';
		read_DATA_X_LSB[2] = '\x2E';
		read_DATA_Y_MSB[2] = '\x31';
		read_DATA_Y_LSB[2] = '\x30';
		read_DATA_Z_MSB[2] = '\x33';
		read_DATA_Z_LSB[2] = '\x32';
		printf("GRV: \n");
	}
	
	uint8_t vec[2];

	// x
	write_to_channel(read_DATA_X_MSB,4);
	vec[1] = read_from_channel();
	//printf("LSB: %x\n", vec[1]);
	//ros::Duration(0.1).sleep();

	write_to_channel(read_DATA_X_LSB,4);
	vec[0] = read_from_channel();
	//printf("MSB: %x\n", vec[0]);
	//ros::Duration(0.1).sleep();

	printf("x: %d\n",convert_to_bytes(vec));

	// y
	write_to_channel(read_DATA_Y_MSB,4);
	vec[1] = read_from_channel();
	//printf("LSB: %x\n", vec[1]);
	//ros::Duration(0.1).sleep();

	write_to_channel(read_DATA_Y_LSB,4);
	vec[0] = read_from_channel();
	//printf("MSB: %x\n", vec[0]);
	//ros::Duration(0.1).sleep();

	printf("y: %d\n",convert_to_bytes(vec));

	// z
	write_to_channel(read_DATA_Z_MSB,4);
	vec[1] = read_from_channel();
	//printf("LSB: %x\n", vec[1]);
	//ros::Duration(0.1).sleep();

	write_to_channel(read_DATA_Z_LSB,4);
	vec[0] = read_from_channel();
	//printf("MSB: %x\n", vec[0]);
	//ros::Duration(0.1).sleep();

	printf("z: %d\n\n",convert_to_bytes(vec));
	//ros::Duration(1).sleep();
}