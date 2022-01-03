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
		// for(auto f : features)
		// {
		// 	read_all_data_UART(f);
		// }
		read_all_data_UART("ACC");
	}
	else if(option == "I2C")
	{
		for(auto f : features)
		{
			read_all_data_I2C(f);
		}
		// read_all_data_I2C("TEMP");
	}
}

int BNO055_IMU::convert_to_bytes(uint8_t vec[2])
{
	uint16_t wd = ((uint16_t)vec[0] << 8) | vec[1];
	// ROS_INFO("%x",wd);
	return (int16_t)wd;
}


uint8_t BNO055_IMU::get_byte(uint8_t* cmd)
{
	send_command(cmd);
	ros::Duration(0.1).sleep();
	uint8_t bte1 = communication->read_from_channel(); // ROS_INFO("bte1: %x",bte1); 
	ros::Duration(0.1).sleep();

	if(bte1 == 0xbb){
		uint8_t bte2 = communication->read_from_channel(); //ROS_INFO("bte2: %x",bte2); 
		ros::Duration(0.1).sleep();
		uint8_t bte3 = communication->read_from_channel(); //ROS_INFO("bte3: %x",bte3); 
		ros::Duration(0.1).sleep();
		return bte3;
	}
	else if(bte1 == 0xee)
	{
		uint8_t bte2 = communication->read_from_channel(); ros::Duration(0.01).sleep();
		// return bte2;
	}
	
	return 0x00;
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
		for(size_t i = 0; i < 6; i++)
		{
			registers.at(i)[2] = nr++; // NO error here
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
		
		//ROS_INFO("MAG: \n");
	}
	else if(opt == "GYR")
	{
		uint8_t nr = GYR_START_BYTE;

		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[2] = nr++;
		}
		
		//ROS_INFO("GYR: \n");
	}
	else if(opt == "EUL")
	{
		uint8_t nr = EUL_START_BYTE;

		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[2] = nr++;
		}
		
		//ROS_INFO("EUL: \n");
	}
	else if(opt == "QUA")
	{
		uint8_t nr = QUA_START_BYTE;

		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[2] = nr++;
		}
		
		//ROS_INFO("QUA: \n");

		// w
		communication->write_to_channel(read_DATA_W_MSB,4);
		vec4[1] = communication->read_from_channel();
		
		communication->write_to_channel(read_DATA_W_LSB,4);
		vec4[0] = communication->read_from_channel();
		
		//ROS_INFO("w: %d\n",convert_to_bytes(vec4));
	}
	else if(opt == "LIA")
	{
		uint8_t nr = LIA_START_BYTE;

		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[2] = nr++;
		}
		
		//ROS_INFO("LIA: \n");
	}
	else if(opt == "GRV")
	{
		uint8_t nr = GRV_START_BYTE;

		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[2] = nr++;
		}
		
		//ROS_INFO("GRV: \n");
	}
	else if(opt == "TEMP")
	{
		uint8_t temp_command[4] = {0xaa,0x1,0x34,0x1};
		send_command(temp_command);
		ros::Duration(0.01).sleep();
		uint8_t tmp = communication->read_from_channel();
		// ROS_INFO("TEMP: %d\n\n", tmp);
		return;
	}

	uint8_t vec[2] = {0};
	uint8_t vec2[2] = {0};
	uint8_t vec3[2] = {0};
 
	// x
	// msb
	vec[0] = get_byte(registers.at(1));
	ROS_INFO("x msb: %x",vec[0]);
	ros::Duration(0.01).sleep();

    // lsb
	vec[1] = get_byte(registers.at(0));
	ROS_INFO("x lsb: %x",vec[1]);
	ros::Duration(0.01).sleep();

	ROS_INFO("x: %d\n",convert_to_bytes(vec));

	// y
	// msb
	vec2[0] = get_byte(registers.at(3));
	ROS_INFO("y msb: %x",vec2[0]);
	ros::Duration(0.01).sleep();

	// lsb
	vec2[1] = get_byte(registers.at(2));
	ROS_INFO("y lsb: %x",vec2[1]);
	ros::Duration(0.01).sleep();

	ROS_INFO("y: %d\n",convert_to_bytes(vec2));

	// z
	// msb
	vec3[0] = get_byte(registers.at(5));
	ROS_INFO("z msb: %x",vec2[0]);
	ros::Duration(0.01).sleep();

	// lsb
	vec3[1] = get_byte(registers.at(4));
	ROS_INFO("z lsb: %x",vec2[0]);
	ros::Duration(0.01).sleep();
	
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

	int divider = 1;

	if(opt == "ACC")
	{
		uint8_t nr = ACC_START_BYTE;
		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[0] = nr++;
		}

		ROS_INFO("ACC: ");
		divider = 100;

	}
	else if(opt == "MAG")
	{
		uint8_t nr = MAG_START_BYTE;
		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[0] = nr++;
		}

		ROS_INFO("MAG: ");
		divider = 16;
	}
	else if(opt == "GYR")
	{
		uint8_t nr = GYR_START_BYTE;
		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[0] = nr++;
		}

		ROS_INFO("GYR: ");
		divider = 16;
	}
	else if(opt == "EUL")
	{
		uint8_t nr = EUL_START_BYTE;
		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[0] = nr++;
		}

		ROS_INFO("EUL: ");
		divider = 16; // for degrees is 16, for radians is 900
	}
	else if(opt == "QUA")
	{
		uint8_t nr = QUA_START_BYTE;
		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[0] = nr++;
		}

		divider = 16384;

		uint8_t read_W_MSB_command[] = {0x21};
		communication->write_to_channel(read_W_MSB_command, 1);
		arr0[0] = communication->read_from_channel();

		uint8_t read_W_LSB_command[] = {0x20};
		communication->write_to_channel(read_W_LSB_command, 1);
		arr0[1] = communication->read_from_channel();

		ROS_INFO("QUA: w = %.4f\n", (float)convert_to_bytes(arr0)/divider);
	}
	else if(opt == "LIA")
	{
		uint8_t nr = LIA_START_BYTE;
		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[0] = nr++;
		}

		ROS_INFO("LIA: ");
		divider = 100; // for m/s^2
	}
	else if(opt == "GRV")
	{
		uint8_t nr = GRV_START_BYTE;
		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[0] = nr++;
		}

		ROS_INFO("GRV: ");
		divider = 100; // for m/s^2
	}
	else if(opt == "TEMP")
	{
		uint8_t read_temp_command[] = {0x34};
		send_command(read_temp_command);
		uint8_t tmp = communication->read_from_channel();
		ROS_INFO("TEMP: %d\n\n", tmp);
		bno055_driver::tmp tmp_msg;
		tmp_msg.data = (float)tmp;
		pub_tmp.publish(tmp_msg);
		return;
	}

	send_command(registers.at(1)); //msb x
	arr[0] = communication->read_from_channel();

	send_command(registers.at(0)); //lsb x
	arr[1] = communication->read_from_channel();

	send_command(registers.at(3)); //msb y
	arr1[0] = communication->read_from_channel();

	send_command(registers.at(2)); //lsb y
	arr1[1] = communication->read_from_channel();

	send_command(registers.at(5)); //msb z
	arr2[0] = communication->read_from_channel();

	send_command(registers.at(4)); //lsb z
	arr2[1] = communication->read_from_channel();

	// ROS_INFO("%x, %x, %x, %x, %x, %x",arr[1], arr[0], arr1[1], arr1[0], arr2[1], arr2[0]);

	ROS_INFO(" (%.4f, %.4f, %.4f)\n",(float)convert_to_bytes(arr)/divider,(float)convert_to_bytes(arr1)/divider,(float)convert_to_bytes(arr2)/divider);

	// publish on topics
	if(opt == "ACC")
	{
		bno055_driver::acc acc_msg;
		acc_msg.x = (float)convert_to_bytes(arr)/divider;
		acc_msg.y = (float)convert_to_bytes(arr1)/divider;
		acc_msg.z = (float)convert_to_bytes(arr2)/divider;
		pub_acc.publish(acc_msg);
	}
	else if(opt == "MAG")
	{
		bno055_driver::mag mag_msg;
		mag_msg.x = (float)convert_to_bytes(arr) /divider;
		mag_msg.y = (float)convert_to_bytes(arr1)/divider;
		mag_msg.z = (float)convert_to_bytes(arr2)/divider;
		pub_mag.publish(mag_msg);
	}
	else if(opt == "GYR")
	{
		bno055_driver::gyr gyr_msg;
		gyr_msg.x = (float)convert_to_bytes(arr) /divider;
		gyr_msg.y = (float)convert_to_bytes(arr1)/divider;
		gyr_msg.z = (float)convert_to_bytes(arr2)/divider;
		pub_gyr.publish(gyr_msg);
	}
	else if(opt == "EUL")
	{
		bno055_driver::eul eul_msg;
		eul_msg.x = (float)convert_to_bytes(arr) /divider;
		eul_msg.y = (float)convert_to_bytes(arr1)/divider;
		eul_msg.z = (float)convert_to_bytes(arr2)/divider;
		pub_eul.publish(eul_msg);
	}
	else if(opt == "QUA")
	{
		bno055_driver::qua qua_msg;
		qua_msg.x = (float)convert_to_bytes(arr) /divider;
		qua_msg.y = (float)convert_to_bytes(arr1)/divider;
		qua_msg.z = (float)convert_to_bytes(arr2)/divider;
		qua_msg.w = (float)convert_to_bytes(arr0)/divider;
		pub_qua.publish(qua_msg);
		
	}
	else if(opt == "LIA")
	{
		bno055_driver::lia lia_msg;
		lia_msg.x = (float)convert_to_bytes(arr) /divider;
		lia_msg.y = (float)convert_to_bytes(arr1)/divider;
		lia_msg.z = (float)convert_to_bytes(arr2)/divider;
		pub_lia.publish(lia_msg);
	}
	else if(opt == "GRV")
	{
		bno055_driver::grv grv_msg;
		grv_msg.x = (float)convert_to_bytes(arr) /divider;
		grv_msg.y = (float)convert_to_bytes(arr1)/divider;
		grv_msg.z = (float)convert_to_bytes(arr2)/divider;
		pub_grv.publish(grv_msg);
	}

	ros::Duration(0.1).sleep();
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
	// ROS_INFO("Size of array: %d", sizeof(command));

	// ROS_INFO("%x %x %x %x ",command[0],command[1],command[2],command[3]);

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

	ros::Rate rate(50);

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
			ros::Duration(0.01).sleep();

			uint8_t OPR_MODE_COMMAND[5] = {0xaa, 0x0, 0x3d, 0x1, 0x7}; // AMG
			bno.get_communication()->write_to_channel(OPR_MODE_COMMAND,5);

			ros::Duration(0.01).sleep();
			uint8_t bte1 = bno.get_communication()->read_from_channel();
			uint8_t bte2 = bno.get_communication()->read_from_channel();
			
			ROS_INFO("\n\n%x ; %x\n\n", bte1, bte2);
			ros::Duration(0.01).sleep();

			bno.set_option("UART");

			while(ros::ok())
			{
				bno.read_data(nullptr,0);
				ros::Duration(0.01).sleep();
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

			Communication* c = cf.create_communication("I2C", "/dev/i2c-1", 0x28);
			// Communication* c = cf.create_communication("I2C", (char*)i2c_port_name.c_str(), address);

			BNO055_IMU bno = BNO055_IMU(&nh,c);

			bno.start_communication();
			ros::Duration(1).sleep();
			
			uint8_t write_amg_cmd[3] = {0x28, 0x3d, 0x07}; //MIGHT BE INCORRECT
			bno.get_communication()->write_to_channel(write_amg_cmd,3);
			ros::Duration(0.01).sleep();

			// set TEMP_SOURCE to have Accel as source
			uint8_t SET_TEMP_SOURCE[2] = {0x40,0x0};
			bno.get_communication()->write_to_channel(SET_TEMP_SOURCE,2);
			ros::Duration(0.01).sleep();

			bno.set_option("I2C");

			while(ros::ok())
			{
				bno.read_data(nullptr,0);
				ros::Duration(0.1).sleep();
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
