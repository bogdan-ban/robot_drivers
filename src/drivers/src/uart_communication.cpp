#include "drivers/uart_communication.h"

UartCommunication::UartCommunication(char* port_name): port(port_name)
{
	start_bno055();
}

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

int UartCommunication::start_bno055()
{
	file_desc = open(port, O_RDWR);

    struct termios tty;

    if(tcgetattr(file_desc, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return 1;
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10; // it was 10    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    if (tcsetattr(file_desc, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return 1;
    }
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

void UartCommunication::flush()
{
	ioctl(file_desc, TCFLSH, 2);
}
