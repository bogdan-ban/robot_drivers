#include "bno055_driver/bno055_imu.h"

BNO055_IMU::BNO055_IMU(ros::NodeHandle* nh, Communication* c) : Sensor(c)
{
	pub_acc = nh->advertise<bno055_driver::acc>(PUBLISH_TOPIC_ACC, 1000);
	pub_mag = nh->advertise<bno055_driver::mag>(PUBLISH_TOPIC_MAG, 1000);
	pub_gyr = nh->advertise<bno055_driver::gyr>(PUBLISH_TOPIC_GYR, 1000);
	pub_eul = nh->advertise<bno055_driver::eul>(PUBLISH_TOPIC_EUL, 1000);
	pub_qua = nh->advertise<bno055_driver::qua>(PUBLISH_TOPIC_QUA, 1000);
	pub_lia = nh->advertise<bno055_driver::lia>(PUBLISH_TOPIC_LIA, 1000);
	pub_grv = nh->advertise<bno055_driver::grv>(PUBLISH_TOPIC_GRV, 1000);
	pub_tmp = nh->advertise<bno055_driver::tmp>(PUBLISH_TOPIC_TMP, 1000);
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

	uint8_t read_DATA_X_MSB[4] = { 0xAA, 0x01, 0x0, 0x01};
	uint8_t read_DATA_X_LSB[4] = { 0xAA, 0x01, 0x0, 0x01};
	uint8_t read_DATA_Y_MSB[4] = { 0xAA, 0x01, 0x0, 0x01};
	uint8_t read_DATA_Y_LSB[4] = { 0xAA, 0x01, 0x0, 0x01};
	uint8_t read_DATA_Z_MSB[4] = { 0xAA, 0x01, 0x0, 0x01};
	uint8_t read_DATA_Z_LSB[4] = { 0xAA, 0x01, 0x0, 0x01};


	std::vector<uint8_t*> registers = {
										read_DATA_X_LSB, 
										read_DATA_X_MSB, 
										read_DATA_Y_LSB, 
										read_DATA_Y_MSB, 
										read_DATA_Z_LSB, 
										read_DATA_Z_MSB
									  };

	// for quaternion
	uint8_t read_DATA_W_MSB[4] = { 0xAA, 0x01, 0x21, 0x01};
	uint8_t read_DATA_W_LSB[4] = { 0xAA, 0x01, 0x20, 0x01};
	uint8_t vec4[2];


	if(opt == "ACC")
	{
		uint8_t nr = ACC_START_BYTE;
		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[2] = nr++;
		}
		
		ROS_INFO("ACC: \n");
	}
	else if(opt == "MAG")
	{
		uint8_t nr = MAG_START_BYTE;

		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[2] = nr++;
		}
		
		ROS_INFO("MAG: \n");
	}
	else if(opt == "GYR")
	{
		uint8_t nr = GYR_START_BYTE;

		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[2] = nr++;
		}
		
		ROS_INFO("GYR: \n");
	}
	else if(opt == "EUL")
	{
		uint8_t nr = EUL_START_BYTE;

		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[2] = nr++;
		}
		
		ROS_INFO("EUL: \n");
	}
	else if(opt == "QUA")
	{
		uint8_t nr = QUA_START_BYTE;

		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[2] = nr++;
		}
		
		ROS_INFO("QUA: \n");

		// w
		communication->write_to_channel(read_DATA_W_MSB,4);
		vec4[1] = communication->read_from_channel();
		
		communication->write_to_channel(read_DATA_W_LSB,4);
		vec4[0] = communication->read_from_channel();
		
		ROS_INFO("w: %d\n",convert_to_bytes(vec4));
	}
	else if(opt == "LIA")
	{
		uint8_t nr = LIA_START_BYTE;

		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[2] = nr++;
		}
		
		ROS_INFO("LIA: \n");
	}
	else if(opt == "GRV")
	{
		uint8_t nr = GRV_START_BYTE;

		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[2] = nr++;
		}
		
		ROS_INFO("GRV: \n");
	}
	else if(opt == "TEMP")
	{
		uint8_t temp_command[4] = {0xaa,0x1,0x34,0x1};
		send_command(temp_command);
		uint8_t tmp = communication->read_from_channel();
		ROS_INFO("TEMP: %d\n\n", tmp);
		return;
	}

	uint8_t vec[2];
	uint8_t vec2[2];
	uint8_t vec3[2];

	// x
	send_command(registers.at(1));
	vec[1] = communication->read_from_channel();

	send_command(registers.at(0));
	vec[0] = communication->read_from_channel();

	ROS_INFO("x: %d\n",convert_to_bytes(vec));

	// y
	send_command(registers.at(3));
	vec2[1] = communication->read_from_channel();

	send_command(registers.at(2));
	vec2[0] = communication->read_from_channel();
	
	ROS_INFO("y: %d\n",convert_to_bytes(vec2));

	// z
	send_command(registers.at(5));
	vec3[1] = communication->read_from_channel();
	
	send_command(registers.at(4));
	vec3[0] = communication->read_from_channel();
	
	ROS_INFO("z: %d\n\n",convert_to_bytes(vec3));

	// publish on topics
	if(opt == "ACC")
	{
		bno055_driver::acc acc_msg;
		acc_msg.x = convert_to_bytes(vec);
		acc_msg.y = convert_to_bytes(vec2);
		acc_msg.z = convert_to_bytes(vec3);
		pub_acc.publish(acc_msg);
	}
	else if(opt == "MAG")
	{
		bno055_driver::mag mag_msg;
		mag_msg.x = convert_to_bytes(vec);
		mag_msg.y = convert_to_bytes(vec2);
		mag_msg.z = convert_to_bytes(vec3);
		pub_mag.publish(mag_msg);
	}
	else if(opt == "GYR")
	{
		bno055_driver::gyr gyr_msg;
		gyr_msg.x = convert_to_bytes(vec);
		gyr_msg.y = convert_to_bytes(vec2);
		gyr_msg.z = convert_to_bytes(vec3);
		pub_gyr.publish(gyr_msg);
	}
	else if(opt == "EUL")
	{
		bno055_driver::eul eul_msg;
		eul_msg.x = convert_to_bytes(vec);
		eul_msg.y = convert_to_bytes(vec2);
		eul_msg.z = convert_to_bytes(vec3);
		pub_eul.publish(eul_msg);
	}
	else if(opt == "QUA")
	{
		bno055_driver::qua qua_msg;
		qua_msg.x = convert_to_bytes(vec);
		qua_msg.y = convert_to_bytes(vec2);
		qua_msg.z = convert_to_bytes(vec3);
		qua_msg.w = convert_to_bytes(vec4);
		pub_qua.publish(qua_msg);
		
	}
	else if(opt == "LIA")
	{
		bno055_driver::lia lia_msg;
		lia_msg.x = convert_to_bytes(vec);
		lia_msg.y = convert_to_bytes(vec2);
		lia_msg.z = convert_to_bytes(vec3);
		pub_lia.publish(lia_msg);
	}
	else if(opt == "GRV")
	{
		bno055_driver::grv grv_msg;
		grv_msg.x = convert_to_bytes(vec);
		grv_msg.y = convert_to_bytes(vec2);
		grv_msg.z = convert_to_bytes(vec3);
		pub_grv.publish(grv_msg);
	}
	else if(opt == "TEMP")
	{
		bno055_driver::tmp tmp_msg;
		tmp_msg.data = convert_to_bytes(vec);
		pub_tmp.publish(tmp_msg);
	}

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
		uint8_t nr = ACC_START_BYTE;
		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[0] = nr++;
		}

		ROS_INFO("ACC: ");

	}
	else if(opt == "MAG")
	{
		uint8_t nr = MAG_START_BYTE;
		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[0] = nr++;
		}

		ROS_INFO("MAG: ");
	}
	else if(opt == "GYR")
	{
		uint8_t nr = GYR_START_BYTE;
		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[0] = nr++;
		}

		ROS_INFO("GYR: ");
	}
	else if(opt == "EUL")
	{
		uint8_t nr = EUL_START_BYTE;
		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[0] = nr++;
		}

		ROS_INFO("EUL: ");

	}
	else if(opt == "QUA")
	{
		uint8_t nr = QUA_START_BYTE;
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

		ROS_INFO("QUA: w = %d\n", convert_to_bytes(arr0));
	}
	else if(opt == "LIA")
	{
		uint8_t nr = LIA_START_BYTE;
		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[0] = nr++;
		}

		ROS_INFO("LIA: ");
	}
	else if(opt == "GRV")
	{
		uint8_t nr = GRV_START_BYTE;
		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[0] = nr++;
		}

		ROS_INFO("GRV: ");

	}
	else if(opt == "TEMP")
	{
		uint8_t read_temp_command[] = {0x34};
		send_command(read_temp_command);
		uint8_t tmp = communication->read_from_channel();
		ROS_INFO("TEMP: %d\n\n", tmp);
		return;
	}

	send_command(registers.at(1));
	arr[0] = communication->read_from_channel();

	send_command(registers.at(0));
	arr[1] = communication->read_from_channel();

	send_command(registers.at(3));
	arr1[0] = communication->read_from_channel();

	send_command(registers.at(2));
	arr1[1] = communication->read_from_channel();

	send_command(registers.at(5));
	arr2[0] = communication->read_from_channel();

	send_command(registers.at(4));
	arr2[1] = communication->read_from_channel();

	ROS_INFO("\n%d, %d, %d, %d, %d, %d\n",arr[1], arr[0],arr1[1], arr1[0],arr2[1], arr2[0]);

	ROS_INFO(" (%d, %d, %d)\n",convert_to_bytes(arr),convert_to_bytes(arr1),convert_to_bytes(arr2));
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

	int comm_type = -1;

	nh.getParam("communication_type",comm_type);
	ROS_INFO("communication_type: %d\n",comm_type);

	switch(comm_type)
	{
		case 0:
		{
			string uart_port_name;
			int baudrate = 0;
			nh.getParam("uart_port_name",uart_port_name);
			nh.getParam("baudrate",baudrate);

			// Communication* c = cf.create_communication("UART", "/dev/ttyS0", 115200);
			Communication* c = cf.create_communication("UART", (char*)uart_port_name.c_str(), baudrate);

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
				ros::Duration(0.1).sleep();
			}

			bno.stop_communication();

		}
		break;

		case 1:
		{
			string i2c_port_name;
			int address = 0;
			nh.getParam("i2c_port_name",i2c_port_name);
			nh.getParam("address",address);

			// Communication* c = cf.create_communication("I2C", "/dev/i2c-2", 0x50);
			Communication* c = cf.create_communication("I2C", (char*)i2c_port_name.c_str(), address);

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
