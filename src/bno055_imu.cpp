#include "robot_drivers/bno055_imu.h"

BNO055_IMU::BNO055_IMU(ros::NodeHandle* nh, Communication* c) : Sensor(c)
{

}

void BNO055_IMU::start_communication()
{
	communication->start();
}

void BNO055_IMU::read_data(uint8_t* buffer, int size)
{
	if(option == "UART")
	{
		for(auto f : features)
		{
			read_all_data_UART(f);
		}
	}
	else if(option == "I2C")
	{
		for(auto f : features)
		{
			read_all_data_I2C(f);
		}
	}
}

int BNO055_IMU::convert_to_bytes(uint8_t vec[2])
{
	return 256 * vec[0] + vec[1];
}

void BNO055_IMU::read_all_data_UART(const string opt)
{

	uint8_t read_DATA_X_MSB[4] = { 0xAA, 0x01, 0x0, 0x01}; // 1
	uint8_t read_DATA_X_LSB[4] = { 0xAA, 0x01, 0x0, 0x01}; // 2
	uint8_t read_DATA_Y_MSB[4] = { 0xAA, 0x01, 0x0, 0x01}; // 3
	uint8_t read_DATA_Y_LSB[4] = { 0xAA, 0x01, 0x0, 0x01}; // 4
	uint8_t read_DATA_Z_MSB[4] = { 0xAA, 0x01, 0x0, 0x01}; // 5
	uint8_t read_DATA_Z_LSB[4] = { 0xAA, 0x01, 0x0, 0x01}; // 6
	

	std::vector<uint8_t*> registers = {read_DATA_X_LSB, read_DATA_X_MSB, read_DATA_Y_LSB, read_DATA_Y_MSB, read_DATA_Z_LSB, read_DATA_Z_MSB};

	// for quaternion
	uint8_t read_DATA_W_MSB[4] = { 0xAA, 0x01, 0x21, 0x01};
	uint8_t read_DATA_W_LSB[4] = { 0xAA, 0x01, 0x20, 0x01};

	if(opt == "ACC")
	{
		uint8_t nr = 0x08;
		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[2] = nr++;
		}

		// read_DATA_X_MSB[2] = '\x09';
		// read_DATA_X_LSB[2] = '\x08';
		// read_DATA_Y_MSB[2] = '\x0B';
		// read_DATA_Y_LSB[2] = '\x0A';
		// read_DATA_Z_MSB[2] = '\x0D';
		// read_DATA_Z_LSB[2] = '\x0C';
		printf("ACC: \n");
	}
	else if(opt == "MAG")
	{
		uint8_t nr = 0x0E;
		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[2] = nr++;
		}
		/*
		read_DATA_X_MSB[2] = '\x0F';
		read_DATA_X_LSB[2] = '\x0E';
		read_DATA_Y_MSB[2] = '\x11';
		read_DATA_Y_LSB[2] = '\x10';
		read_DATA_Z_MSB[2] = '\x13';
		read_DATA_Z_LSB[2] = '\x12';
		*/
		printf("MAG: \n");
	}
	else if(opt == "GYR")
	{
		uint8_t nr = 0x14;
		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[2] = nr++;
		}
		/*
		read_DATA_X_MSB[2] = '\x15';
		read_DATA_X_LSB[2] = '\x14';
		read_DATA_Y_MSB[2] = '\x17';
		read_DATA_Y_LSB[2] = '\x16';
		read_DATA_Z_MSB[2] = '\x19';
		read_DATA_Z_LSB[2] = '\x18';
		*/
		printf("GYR: \n");
	}
	else if(opt == "EUL")
	{
		uint8_t nr = 0x1A;
		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[2] = nr++;
		}
		/*
		read_DATA_X_MSB[2] = '\x1B';
		read_DATA_X_LSB[2] = '\x1A';
		read_DATA_Y_MSB[2] = '\x1D';
		read_DATA_Y_LSB[2] = '\x1C';
		read_DATA_Z_MSB[2] = '\x1F';
		read_DATA_Z_LSB[2] = '\x1E';
		*/
		printf("EUL: \n");
	}
	else if(opt == "QUA")
	{
		uint8_t nr = 0x22;
		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[2] = nr++;
		}
		/*
		read_DATA_X_MSB[2] = '\x23';
		read_DATA_X_LSB[2] = '\x22';
		read_DATA_Y_MSB[2] = '\x25';
		read_DATA_Y_LSB[2] = '\x24';
		read_DATA_Z_MSB[2] = '\x27';
		read_DATA_Z_LSB[2] = '\x26';
		*/
		printf("QUA: \n");
		// w
		uint8_t vec[2];
		communication->write_to_channel(read_DATA_W_MSB,4);
		vec[1] = communication->read_from_channel();
		//printf("LSB: %x\n", vec[1]);
		//ros::Duration(0.1).sleep();

		communication->write_to_channel(read_DATA_W_LSB,4);
		vec[0] = communication->read_from_channel();
		//printf("MSB: %x\n", vec[0]);
		//ros::Duration(0.1).sleep();

		printf("w: %d\n",convert_to_bytes(vec));
	}
	else if(opt == "LIA")
	{
		uint8_t nr = 0x08;
		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[2] = nr++;
		}
		/*
		read_DATA_X_MSB[2] = '\x29';
		read_DATA_X_LSB[2] = '\x28';
		read_DATA_Y_MSB[2] = '\x2B';
		read_DATA_Y_LSB[2] = '\x2A';
		read_DATA_Z_MSB[2] = '\x2D';
		read_DATA_Z_LSB[2] = '\x2C';
		*/
		printf("LIA: \n");
	}
	else if(opt == "GRV")
	{
		uint8_t nr = 0x08;
		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[2] = nr++;
		}
		/*
		read_DATA_X_MSB[2] = '\x2F';
		read_DATA_X_LSB[2] = '\x2E';
		read_DATA_Y_MSB[2] = '\x31';
		read_DATA_Y_LSB[2] = '\x30';
		read_DATA_Z_MSB[2] = '\x33';
		read_DATA_Z_LSB[2] = '\x32';
		*/
		printf("GRV: \n");
	}
		
	uint8_t vec[2];

	// x
	// communication->write_to_channel(read_DATA_X_MSB,4);
	// communication->write_to_channel(registers.at(1),4);
	send_command(registers.at(1));
	vec[1] = communication->read_from_channel();
	//printf("LSB: %x\n", vec[1]);
	//ros::Duration(0.1).sleep();

	// communication->write_to_channel(read_DATA_X_LSB,4);
	// communication->write_to_channel(registers.at(0),4);
	send_command(registers.at(0));
	vec[0] = communication->read_from_channel();
	//printf("MSB: %x\n", vec[0]);
	//ros::Duration(0.1).sleep();

	printf("x: %d\n",convert_to_bytes(vec));

	// y
	// communication->write_to_channel(read_DATA_Y_MSB,4);
	// communication->write_to_channel(registers.at(3),4);
	send_command(registers.at(3));
	vec[1] = communication->read_from_channel();
	//printf("LSB: %x\n", vec[1]);
	//ros::Duration(0.1).sleep();

	// communication->write_to_channel(read_DATA_Y_LSB,4);
	// communication->write_to_channel(registers.at(2),4);
	send_command(registers.at(2));
	vec[0] = communication->read_from_channel();
	//printf("MSB: %x\n", vec[0]);
	//ros::Duration(0.1).sleep();

	printf("y: %d\n",convert_to_bytes(vec));

	// z
	// communication->write_to_channel(read_DATA_Z_MSB,4);
	// communication->write_to_channel(registers.at(5),4);
	send_command(registers.at(5));
	vec[1] = communication->read_from_channel();
	//printf("LSB: %x\n", vec[1]);
	//ros::Duration(0.1).sleep();

	// communication->write_to_channel(read_DATA_Z_LSB,4);
	// communication->write_to_channel(registers.at(4),4);
	send_command(registers.at(4));
	vec[0] = communication->read_from_channel();
	//printf("MSB: %x\n", vec[0]);
	//ros::Duration(0.1).sleep();

	printf("z: %d\n\n",convert_to_bytes(vec));
	//ros::Duration(1).sleep();
}

void BNO055_IMU::read_all_data_I2C(const string opt)
{
	uint8_t arr[2];
	uint8_t arr1[2];
	uint8_t arr2[2];
	uint8_t arr0[2]; // used only for quaternion: w

	uint8_t read_X_MSB_command[] = {0x00};
	uint8_t read_X_LSB_command[] = {0x00};
	uint8_t read_Y_MSB_command[] = {0x00};
	uint8_t read_Y_LSB_command[] = {0x00};
	uint8_t read_Z_MSB_command[] = {0x00};
	uint8_t read_Z_LSB_command[] = {0x00};

	std::vector<uint8_t*> registers = {read_X_LSB_command, read_X_MSB_command,
									   read_Y_LSB_command, read_Y_MSB_command, 
									   read_Z_LSB_command, read_Z_MSB_command};

	if(opt == "ACC")
	{
		uint8_t nr = 0x08; // #define ACC_START
		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[0] = nr++;
		}

		/*uint8_t read_X_MSB_command[] = {0x9};
		uint8_t read_X_LSB_command[] = {0x8};
		
		uint8_t read_Y_MSB_command[] = {0xB};
		
		uint8_t read_Y_LSB_command[] = {0xA};
		
		uint8_t read_Z_MSB_command[] = {0xD};
		
		uint8_t read_Z_LSB_command[] = {0xC};
		*/

		// printf("ACC: (%d, %d, %d)\n",convert_to_bytes(arr),convert_to_bytes(arr1),convert_to_bytes(arr2));
		printf("ACC: ");

	}
	else if(opt == "MAG")
	{
		uint8_t nr = 0xE;
		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[0] = nr++;
		}
		
		printf("MAG: ");

		/*
		uint8_t read_X_MSB_command[] = {0xF};
		communication->write_to_channel(read_X_MSB_command, 1);
		arr[0] = communication->read_from_channel();

		uint8_t read_X_LSB_command[] = {0xE};
		communication->write_to_channel(read_X_LSB_command, 1);
		arr[1] = communication->read_from_channel();

		uint8_t read_Y_MSB_command[] = {0x11};
		communication->write_to_channel(read_Y_MSB_command, 1);
		arr1[0] = communication->read_from_channel();

		uint8_t read_Y_LSB_command[] = {0x10};
		communication->write_to_channel(read_Y_LSB_command, 1);
		arr1[1] = communication->read_from_channel();

		uint8_t read_Z_MSB_command[] = {0x13};
		communication->write_to_channel(read_Z_MSB_command, 1);
		arr2[0] = communication->read_from_channel();

		uint8_t read_Z_LSB_command[] = {0x12};
		communication->write_to_channel(read_Z_LSB_command, 1);
		arr2[1] = communication->read_from_channel();

		printf("MAG: (%d, %d, %d)\n",convert_to_bytes(arr),convert_to_bytes(arr1),convert_to_bytes(arr2));
		*/
		
	}
	else if(opt == "GYR")
	{
		uint8_t nr = 0x14;
		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[0] = nr++;
		}
		
		printf("GYR: ");

		/*uint8_t read_X_MSB_command[] = {0x15};
		communication->write_to_channel(read_X_MSB_command, 1);
		arr[0] = communication->read_from_channel();

		uint8_t read_X_LSB_command[] = {0x14};
		communication->write_to_channel(read_X_LSB_command, 1);
		arr[1] = communication->read_from_channel();

		uint8_t read_Y_MSB_command[] = {0x17};
		communication->write_to_channel(read_Y_MSB_command, 1);
		arr1[0] = communication->read_from_channel();

		uint8_t read_Y_LSB_command[] = {0x16};
		communication->write_to_channel(read_Y_LSB_command, 1);
		arr1[1] = communication->read_from_channel();

		uint8_t read_Z_MSB_command[] = {0x19};
		communication->write_to_channel(read_Z_MSB_command, 1);
		arr2[0] = communication->read_from_channel();

		uint8_t read_Z_LSB_command[] = {0x18};
		communication->write_to_channel(read_Z_LSB_command, 1);
		arr2[1] = communication->read_from_channel();

		printf("GYR: (%d, %d, %d)\n",convert_to_bytes(arr),convert_to_bytes(arr1),convert_to_bytes(arr2));
		*/
	}
	else if(opt == "EUL")
	{
		uint8_t nr = 0x1A;
		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[0] = nr++;
		}
		
		printf("EUL: ");

		/*uint8_t read_X_MSB_command[] = {0x1B};
		communication->write_to_channel(read_X_MSB_command, 1);
		arr[0] = communication->read_from_channel();

		uint8_t read_X_LSB_command[] = {0x1A};
		communication->write_to_channel(read_X_LSB_command, 1);
		arr[1] = communication->read_from_channel();

		uint8_t read_Y_MSB_command[] = {0x1D};
		communication->write_to_channel(read_Y_MSB_command, 1);
		arr1[0] = communication->read_from_channel();

		uint8_t read_Y_LSB_command[] = {0x1C};
		communication->write_to_channel(read_Y_LSB_command, 1);
		arr1[1] = communication->read_from_channel();

		uint8_t read_Z_MSB_command[] = {0x1F};
		communication->write_to_channel(read_Z_MSB_command, 1);
		arr2[0] = communication->read_from_channel();

		uint8_t read_Z_LSB_command[] = {0x1E};
		communication->write_to_channel(read_Z_LSB_command, 1);
		arr2[1] = communication->read_from_channel();

		printf("EUL: (%d, %d, %d)\n",convert_to_bytes(arr),convert_to_bytes(arr1),convert_to_bytes(arr2));
		*/
		
	}
	else if(opt == "QUA")
	{
		uint8_t nr = 0x20;
		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[0] = nr++;
		}

		uint8_t read_W_MSB_command[] = {0x21};
		communication->write_to_channel(read_W_MSB_command, 1);
		arr0[0] = communication->read_from_channel();

		uint8_t read_W_LSB_command[] = {0x20};
		communication->write_to_channel(read_W_LSB_command, 1);
		arr0[1] = communication->read_from_channel();

		printf("QUA: w = %d\n", convert_to_bytes(arr0));

		/*uint8_t read_X_MSB_command[] = {0x23};
		communication->write_to_channel(read_X_MSB_command, 1);
		arr[0] = communication->read_from_channel();

		uint8_t read_X_LSB_command[] = {0x22};
		communication->write_to_channel(read_X_LSB_command, 1);
		arr[1] = communication->read_from_channel();

		uint8_t read_Y_MSB_command[] = {0x25};
		communication->write_to_channel(read_Y_MSB_command, 1);
		arr1[0] = communication->read_from_channel();

		uint8_t read_Y_LSB_command[] = {0x24};
		communication->write_to_channel(read_Y_LSB_command, 1);
		arr1[1] = communication->read_from_channel();

		uint8_t read_Z_MSB_command[] = {0x27};
		communication->write_to_channel(read_Z_MSB_command, 1);
		arr2[0] = communication->read_from_channel();

		uint8_t read_Z_LSB_command[] = {0x26};
		communication->write_to_channel(read_Z_LSB_command, 1);
		arr2[1] = communication->read_from_channel();

		printf("QUA: (%d, %d, %d, %d)\n",convert_to_bytes(arr0),convert_to_bytes(arr),convert_to_bytes(arr1),convert_to_bytes(arr2));
		*/
	}
	else if(opt == "LIA")
	{
		uint8_t nr = 0x28;
		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[0] = nr++;
		}
		
		printf("LIA: ");

		/*uint8_t read_X_MSB_command[] = {0x29};
		communication->write_to_channel(read_X_MSB_command, 1);
		arr[0] = communication->read_from_channel();

		uint8_t read_X_LSB_command[] = {0x28};
		communication->write_to_channel(read_X_LSB_command, 1);
		arr[1] = communication->read_from_channel();

		uint8_t read_Y_MSB_command[] = {0x2B};
		communication->write_to_channel(read_Y_MSB_command, 1);
		arr1[0] = communication->read_from_channel();

		uint8_t read_Y_LSB_command[] = {0x2A};
		communication->write_to_channel(read_Y_LSB_command, 1);
		arr1[1] = communication->read_from_channel();

		uint8_t read_Z_MSB_command[] = {0x2D};
		communication->write_to_channel(read_Z_MSB_command, 1);
		arr2[0] = communication->read_from_channel();

		uint8_t read_Z_LSB_command[] = {0x2C};
		communication->write_to_channel(read_Z_LSB_command, 1);
		arr2[1] = communication->read_from_channel();

		printf("LIA: (%d, %d, %d)\n",convert_to_bytes(arr),convert_to_bytes(arr1),convert_to_bytes(arr2));
		*/
		
	}
	else if(opt == "GRV")
	{
		uint8_t nr = 0x2E;
		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[0] = nr++;
		}
		
		printf("GRV: ");

		/*uint8_t read_X_MSB_command[] = {0x2F};
		communication->write_to_channel(read_X_MSB_command, 1);
		arr[0] = communication->read_from_channel();

		uint8_t read_X_LSB_command[] = {0x2E};
		communication->write_to_channel(read_X_LSB_command, 1);
		arr[1] = communication->read_from_channel();

		uint8_t read_Y_MSB_command[] = {0x31};
		communication->write_to_channel(read_Y_MSB_command, 1);
		arr1[0] = communication->read_from_channel();

		uint8_t read_Y_LSB_command[] = {0x30};
		communication->write_to_channel(read_Y_LSB_command, 1);
		arr1[1] = communication->read_from_channel();

		uint8_t read_Z_MSB_command[] = {0x33};
		communication->write_to_channel(read_Z_MSB_command, 1);
		arr2[0] = communication->read_from_channel();

		uint8_t read_Z_LSB_command[] = {0x32};
		communication->write_to_channel(read_Z_LSB_command, 1);
		arr2[1] = communication->read_from_channel();

		printf("GRV: (%d, %d, %d)\n",convert_to_bytes(arr),convert_to_bytes(arr1),convert_to_bytes(arr2));
		*/
	}
	else if(opt == "TEMP")
	{
		uint8_t read_temp_command[] = {0x34};
		// communication->write_to_channel(read_temp_command, 1);
		send_command(read_temp_command);
		uint8_t tmp = communication->read_from_channel();
		printf("TEMP: %d\n\n", tmp);
	}

	// communication->write_to_channel(read_X_MSB_command, 1);
	send_command(read_X_MSB_command);
	arr[0] = communication->read_from_channel();

	// communication->write_to_channel(read_X_LSB_command, 1);
	send_command(read_X_LSB_command);
	arr[1] = communication->read_from_channel();

	// communication->write_to_channel(read_Y_MSB_command, 1);
	send_command(read_Y_MSB_command);
	arr1[0] = communication->read_from_channel();

	// communication->write_to_channel(read_Y_LSB_command, 1);
	send_command(read_Y_LSB_command);
	arr1[1] = communication->read_from_channel();

	// communication->write_to_channel(read_Z_MSB_command, 1);
	send_command(read_Z_MSB_command);
	arr2[0] = communication->read_from_channel();

	// communication->write_to_channel(read_Z_LSB_command, 1);
	send_command(read_Z_LSB_command);
	arr2[1] = communication->read_from_channel();

	printf(" (%d, %d, %d)\n",convert_to_bytes(arr),convert_to_bytes(arr1),convert_to_bytes(arr2));
}

void BNO055_IMU::set_option(const string _opt)
{
	option = _opt;
}

Communication* BNO055_IMU::get_communication()
{
	return communication;
}

void BNO055_IMU::stop_communication()
{
	communication->stop();
}

void BNO055_IMU::send_command(uint8_t* command)
{
	communication->write_to_channel(command,sizeof(command));
}

void BNO055_IMU::publish_message()
{

}

void BNO055_IMU::create_message()
{

}


int main(int argc, char** argv)
{
	ros::init(argc,argv,"bno055_node");

	ros::NodeHandle nh;

	ros::Rate rate(10);

	CommunicationFactory cf = CommunicationFactory();


	int option = 0;
	cout << "Choose communication: (UART)0, (I2C)1, (SPI)2\n";
	cin >> option;

	switch(option)
	{
		case 0:
		{
			Communication* c = cf.create_communication("UART", "/dev/ttyS0", 115200);

			BNO055_IMU bno = BNO055_IMU(&nh,c);

			bno.start_communication();
			ros::Duration(1).sleep();

			const uint8_t OPR_MODE_COMMAND[5] = {0xaa, 0x00, 0x3d, 0x1, 0x7}; // AMG
			bno.get_communication()->write_to_channel(OPR_MODE_COMMAND,5);
			cout << bno.get_communication()->read_from_channel() << endl;
			ros::Duration(1).sleep();

			bno.set_option("UART");

			while(ros::ok())
			{
				bno.read_data(nullptr,0);
				ros::Duration(1).sleep();
			}

			bno.stop_communication();	

		}
		break;

		case 1:
		{
			Communication* c = cf.create_communication("I2C", "/dev/i2c-2", 0x50);

			BNO055_IMU bno = BNO055_IMU(&nh,c);

			bno.start_communication();
			ros::Duration(1).sleep();

			bno.set_option("I2C");

			while(ros::ok())
			{
				bno.read_data(nullptr,0);
				ros::Duration(1).sleep();
			}

			bno.stop_communication();

		}
		break;

		default:
		cout << "Please choose an option from [0,1] interval";
	}

	ros::spin();

	return 0;
}
