#include "robot_drivers/i2c_communication.h"


I2cCommunication::I2cCommunication(char* port_name, int addr)
{
	// port = (char*)"/dev/i2c-2";
	port = port_name;
	address = addr;
}

int I2cCommunication::start()
{
	if((fd = open(port,O_RDWR)) < 0)
	{
		cout << "open error\n";
		return -1;
	}

	if(ioctl(fd,I2C_SLAVE,address) < 0)
	{
		cout << "ioctl error\n";
		return -1;
	}
	return fd;
}

uint8_t I2cCommunication::read_from_channel()
{
	uint8_t buffer[] = {0x0};
	ssize_t size = sizeof(buffer);

	if(read(fd,buffer,size) != size)
	{
		cout << "failed to read from i2c \n";
		return 0x0;
	}
	else
	{
		//for(int i=0;i<size;i++)
		//	printf("%x ",buffer[i]);
		//cout << endl;
		return buffer[0];
	}
}

void I2cCommunication::write_to_channel(const uint8_t buffer[],int size)
{
	if(write(fd,buffer,size) != size)
	{
		cout << "failed to write to i2c \n";
		return;
	}
}

void I2cCommunication::stop()
{
	close(fd);
}

int I2cCommunication::convert_to_bytes(uint8_t vec[2])
{
	return 256 * vec[0] + vec[1];
}

void I2cCommunication::read_all_data(const string opt)
{
	// int bus = fd;

	uint8_t arr[2];
	uint8_t arr1[2];
	uint8_t arr2[2];
	uint8_t arr0[2]; // used only for quaternion: w

	if(opt == "ACC")
	{
		uint8_t read_X_MSB_command[] = {0x9};
		write_to_channel(read_X_MSB_command, 1);
		arr[0] = read_from_channel();

		uint8_t read_X_LSB_command[] = {0x8};
		write_to_channel(read_X_LSB_command, 1);
		arr[1] = read_from_channel();

		uint8_t read_Y_MSB_command[] = {0xB};
		write_to_channel(read_Y_MSB_command, 1);
		arr1[0] = read_from_channel();

		uint8_t read_Y_LSB_command[] = {0xA};
		write_to_channel(read_Y_LSB_command, 1);
		arr1[1] = read_from_channel();

		uint8_t read_Z_MSB_command[] = {0xD};
		write_to_channel(read_Z_MSB_command, 1);
		arr2[0] = read_from_channel();

		uint8_t read_Z_LSB_command[] = {0xC};
		write_to_channel(read_Z_LSB_command, 1);
		arr2[1] = read_from_channel();

		printf("ACC: (%d, %d, %d)\n",convert_to_bytes(arr),convert_to_bytes(arr1),convert_to_bytes(arr2));

	}else if(opt == "MAG")
	{
		uint8_t read_X_MSB_command[] = {0xF};
		write_to_channel(read_X_MSB_command, 1);
		arr[0] = read_from_channel();

		uint8_t read_X_LSB_command[] = {0xE};
		write_to_channel(read_X_LSB_command, 1);
		arr[1] = read_from_channel();

		uint8_t read_Y_MSB_command[] = {0x11};
		write_to_channel(read_Y_MSB_command, 1);
		arr1[0] = read_from_channel();

		uint8_t read_Y_LSB_command[] = {0x10};
		write_to_channel(read_Y_LSB_command, 1);
		arr1[1] = read_from_channel();

		uint8_t read_Z_MSB_command[] = {0x13};
		write_to_channel(read_Z_MSB_command, 1);
		arr2[0] = read_from_channel();

		uint8_t read_Z_LSB_command[] = {0x12};
		write_to_channel(read_Z_LSB_command, 1);
		arr2[1] = read_from_channel();

		printf("MAG: (%d, %d, %d)\n",convert_to_bytes(arr),convert_to_bytes(arr1),convert_to_bytes(arr2));

	}else if(opt == "GYR")
	{
		uint8_t read_X_MSB_command[] = {0x15};
		write_to_channel(read_X_MSB_command, 1);
		arr[0] = read_from_channel();

		uint8_t read_X_LSB_command[] = {0x14};
		write_to_channel(read_X_LSB_command, 1);
		arr[1] = read_from_channel();

		uint8_t read_Y_MSB_command[] = {0x17};
		write_to_channel(read_Y_MSB_command, 1);
		arr1[0] = read_from_channel();

		uint8_t read_Y_LSB_command[] = {0x16};
		write_to_channel(read_Y_LSB_command, 1);
		arr1[1] = read_from_channel();

		uint8_t read_Z_MSB_command[] = {0x19};
		write_to_channel(read_Z_MSB_command, 1);
		arr2[0] = read_from_channel();

		uint8_t read_Z_LSB_command[] = {0x18};
		write_to_channel(read_Z_LSB_command, 1);
		arr2[1] = read_from_channel();

		printf("GYR: (%d, %d, %d)\n",convert_to_bytes(arr),convert_to_bytes(arr1),convert_to_bytes(arr2));
		
	}else if(opt == "EUL")
	{
		uint8_t read_X_MSB_command[] = {0x1B};
		write_to_channel(read_X_MSB_command, 1);
		arr[0] = read_from_channel();

		uint8_t read_X_LSB_command[] = {0x1A};
		write_to_channel(read_X_LSB_command, 1);
		arr[1] = read_from_channel();

		uint8_t read_Y_MSB_command[] = {0x1D};
		write_to_channel(read_Y_MSB_command, 1);
		arr1[0] = read_from_channel();

		uint8_t read_Y_LSB_command[] = {0x1C};
		write_to_channel(read_Y_LSB_command, 1);
		arr1[1] = read_from_channel();

		uint8_t read_Z_MSB_command[] = {0x1F};
		write_to_channel(read_Z_MSB_command, 1);
		arr2[0] = read_from_channel();

		uint8_t read_Z_LSB_command[] = {0x1E};
		write_to_channel(read_Z_LSB_command, 1);
		arr2[1] = read_from_channel();

		printf("EUL: (%d, %d, %d)\n",convert_to_bytes(arr),convert_to_bytes(arr1),convert_to_bytes(arr2));

		
	}else if(opt == "QUA")
	{
		uint8_t read_W_MSB_command[] = {0x21};
		write_to_channel(read_W_MSB_command, 1);
		arr0[0] = read_from_channel();

		uint8_t read_W_LSB_command[] = {0x20};
		write_to_channel(read_W_LSB_command, 1);
		arr0[1] = read_from_channel();

		uint8_t read_X_MSB_command[] = {0x23};
		write_to_channel(read_X_MSB_command, 1);
		arr[0] = read_from_channel();

		uint8_t read_X_LSB_command[] = {0x22};
		write_to_channel(read_X_LSB_command, 1);
		arr[1] = read_from_channel();

		uint8_t read_Y_MSB_command[] = {0x25};
		write_to_channel(read_Y_MSB_command, 1);
		arr1[0] = read_from_channel();

		uint8_t read_Y_LSB_command[] = {0x24};
		write_to_channel(read_Y_LSB_command, 1);
		arr1[1] = read_from_channel();

		uint8_t read_Z_MSB_command[] = {0x27};
		write_to_channel(read_Z_MSB_command, 1);
		arr2[0] = read_from_channel();

		uint8_t read_Z_LSB_command[] = {0x26};
		write_to_channel(read_Z_LSB_command, 1);
		arr2[1] = read_from_channel();

		printf("QUA: (%d, %d, %d, %d)\n",convert_to_bytes(arr0),convert_to_bytes(arr),convert_to_bytes(arr1),convert_to_bytes(arr2));
		
	}else if(opt == "LIA")
	{
		uint8_t read_X_MSB_command[] = {0x29};
		write_to_channel(read_X_MSB_command, 1);
		arr[0] = read_from_channel();

		uint8_t read_X_LSB_command[] = {0x28};
		write_to_channel(read_X_LSB_command, 1);
		arr[1] = read_from_channel();

		uint8_t read_Y_MSB_command[] = {0x2B};
		write_to_channel(read_Y_MSB_command, 1);
		arr1[0] = read_from_channel();

		uint8_t read_Y_LSB_command[] = {0x2A};
		write_to_channel(read_Y_LSB_command, 1);
		arr1[1] = read_from_channel();

		uint8_t read_Z_MSB_command[] = {0x2D};
		write_to_channel(read_Z_MSB_command, 1);
		arr2[0] = read_from_channel();

		uint8_t read_Z_LSB_command[] = {0x2C};
		write_to_channel(read_Z_LSB_command, 1);
		arr2[1] = read_from_channel();

		printf("LIA: (%d, %d, %d)\n",convert_to_bytes(arr),convert_to_bytes(arr1),convert_to_bytes(arr2));

		
	}else if(opt == "GRV")
	{
		uint8_t read_X_MSB_command[] = {0x2F};
		write_to_channel(read_X_MSB_command, 1);
		arr[0] = read_from_channel();

		uint8_t read_X_LSB_command[] = {0x2E};
		write_to_channel(read_X_LSB_command, 1);
		arr[1] = read_from_channel();

		uint8_t read_Y_MSB_command[] = {0x31};
		write_to_channel(read_Y_MSB_command, 1);
		arr1[0] = read_from_channel();

		uint8_t read_Y_LSB_command[] = {0x30};
		write_to_channel(read_Y_LSB_command, 1);
		arr1[1] = read_from_channel();

		uint8_t read_Z_MSB_command[] = {0x33};
		write_to_channel(read_Z_MSB_command, 1);
		arr2[0] = read_from_channel();

		uint8_t read_Z_LSB_command[] = {0x32};
		write_to_channel(read_Z_LSB_command, 1);
		arr2[1] = read_from_channel();

		printf("GRV: (%d, %d, %d)\n",convert_to_bytes(arr),convert_to_bytes(arr1),convert_to_bytes(arr2));
		
	}else if(opt == "TEMP")
	{
		uint8_t read_temp_command[] = {0x34};
		write_to_channel(read_temp_command, 1);
		uint8_t tmp = read_from_channel();
		printf("TEMP: %d\n\n", tmp);
	}
}