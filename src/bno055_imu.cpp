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
		read_all_data_UART("ACC");
		read_all_data_UART("MAG");
		read_all_data_UART("GYR");
		read_all_data_UART("EUL");
		read_all_data_UART("QUA");
		read_all_data_UART("LIA");
		read_all_data_UART("GRV");
	}else if(option == "I2C")
	{
		read_all_data_I2C("ACC");
		read_all_data_I2C("MAG");
		read_all_data_I2C("GYR");
		read_all_data_I2C("EUL");
		read_all_data_I2C("QUA");
		read_all_data_I2C("LIA");
		read_all_data_I2C("GRV");
	}
}

int BNO055_IMU::convert_to_bytes(uint8_t vec[2])
{
	return 256 * vec[0] + vec[1];
}

void BNO055_IMU::read_all_data_UART(const string opt)
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
	communication->write_to_channel(read_DATA_X_MSB,4);
	vec[1] = communication->read_from_channel();
	//printf("LSB: %x\n", vec[1]);
	//ros::Duration(0.1).sleep();

	communication->write_to_channel(read_DATA_X_LSB,4);
	vec[0] = communication->read_from_channel();
	//printf("MSB: %x\n", vec[0]);
	//ros::Duration(0.1).sleep();

	printf("x: %d\n",convert_to_bytes(vec));

	// y
	communication->write_to_channel(read_DATA_Y_MSB,4);
	vec[1] = communication->read_from_channel();
	//printf("LSB: %x\n", vec[1]);
	//ros::Duration(0.1).sleep();

	communication->write_to_channel(read_DATA_Y_LSB,4);
	vec[0] = communication->read_from_channel();
	//printf("MSB: %x\n", vec[0]);
	//ros::Duration(0.1).sleep();

	printf("y: %d\n",convert_to_bytes(vec));

	// z
	communication->write_to_channel(read_DATA_Z_MSB,4);
	vec[1] = communication->read_from_channel();
	//printf("LSB: %x\n", vec[1]);
	//ros::Duration(0.1).sleep();

	communication->write_to_channel(read_DATA_Z_LSB,4);
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

	if(opt == "ACC")
	{
		uint8_t read_X_MSB_command[] = {0x9};
		communication->write_to_channel(read_X_MSB_command, 1);
		arr[0] = communication->read_from_channel();

		uint8_t read_X_LSB_command[] = {0x8};
		communication->write_to_channel(read_X_LSB_command, 1);
		arr[1] = communication->read_from_channel();

		uint8_t read_Y_MSB_command[] = {0xB};
		communication->write_to_channel(read_Y_MSB_command, 1);
		arr1[0] = communication->read_from_channel();

		uint8_t read_Y_LSB_command[] = {0xA};
		communication->write_to_channel(read_Y_LSB_command, 1);
		arr1[1] = communication->read_from_channel();

		uint8_t read_Z_MSB_command[] = {0xD};
		communication->write_to_channel(read_Z_MSB_command, 1);
		arr2[0] = communication->read_from_channel();

		uint8_t read_Z_LSB_command[] = {0xC};
		communication->write_to_channel(read_Z_LSB_command, 1);
		arr2[1] = communication->read_from_channel();

		printf("ACC: (%d, %d, %d)\n",convert_to_bytes(arr),convert_to_bytes(arr1),convert_to_bytes(arr2));

	}
	else if(opt == "MAG")
	{
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

	}
	else if(opt == "GYR")
	{
		uint8_t read_X_MSB_command[] = {0x15};
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
		
	}
	else if(opt == "EUL")
	{
		uint8_t read_X_MSB_command[] = {0x1B};
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

		
	}
	else if(opt == "QUA")
	{
		uint8_t read_W_MSB_command[] = {0x21};
		communication->write_to_channel(read_W_MSB_command, 1);
		arr0[0] = communication->read_from_channel();

		uint8_t read_W_LSB_command[] = {0x20};
		communication->write_to_channel(read_W_LSB_command, 1);
		arr0[1] = communication->read_from_channel();

		uint8_t read_X_MSB_command[] = {0x23};
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
		
	}
	else if(opt == "LIA")
	{
		uint8_t read_X_MSB_command[] = {0x29};
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

		
	}
	else if(opt == "GRV")
	{
		uint8_t read_X_MSB_command[] = {0x2F};
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
		
	}
	else if(opt == "TEMP")
	{
		uint8_t read_temp_command[] = {0x34};
		communication->write_to_channel(read_temp_command, 1);
		uint8_t tmp = communication->read_from_channel();
		printf("TEMP: %d\n\n", tmp);
	}
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
			Communication* c = cf.create_communication("UART");

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
			Communication* c = cf.create_communication("I2C");

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
