#include "bno055_driver/bno055_imu.h" // ctrl + K + 0/j to collapse/expand to definition

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
	uint16_t wd = ((uint16_t)vec[0] << 8) | vec[1];
	return (int16_t)wd;
}

uint8_t BNO055_IMU::get_byte(uint8_t cmd[])
{
	send_command(cmd);
	ros::Duration(0.01).sleep();
	uint8_t bte1 = communication->read_from_channel();  

	if(bte1 == 0xbb){
		uint8_t bte2 = communication->read_from_channel(); 
		uint8_t bte3 = communication->read_from_channel(); 
		return bte3;
	}
	else if(bte1 == 0xee)
	{
		uint8_t bte2 = communication->read_from_channel();
		return 0x00;
	}
	
	return 0x00;
}

void BNO055_IMU::read_all_data_UART(const string opt)
{

	int divider = 1;
	uint8_t x_coord[2] = {0};
	uint8_t y_coord[2] = {0};
	uint8_t z_coord[2] = {0};
	uint8_t w_coord[2] = {0};
	uint8_t read_DATA_X_MSB[4] = { 0xAA, 0x01, 0x0, 0x01};
	uint8_t read_DATA_X_LSB[4] = { 0xAA, 0x01, 0x0, 0x01};
	uint8_t read_DATA_Y_MSB[4] = { 0xAA, 0x01, 0x0, 0x01};
	uint8_t read_DATA_Y_LSB[4] = { 0xAA, 0x01, 0x0, 0x01};
	uint8_t read_DATA_Z_MSB[4] = { 0xAA, 0x01, 0x0, 0x01};
	uint8_t read_DATA_Z_LSB[4] = { 0xAA, 0x01, 0x0, 0x01};
	// for quaternion
	uint8_t read_DATA_W_MSB[4] = { 0xAA, 0x01, 0x21, 0x01};
	uint8_t read_DATA_W_LSB[4] = { 0xAA, 0x01, 0x20, 0x01};

	std::vector<uint8_t*> registers = {
										read_DATA_X_LSB, 
										read_DATA_X_MSB, 
										read_DATA_Y_LSB, 
										read_DATA_Y_MSB, 
										read_DATA_Z_LSB, 
										read_DATA_Z_MSB
									  };

	if(opt == "ACC")
	{
		uint8_t nr = ACC_START_BYTE;
		for(size_t i = 0; i < 6; i++)
		{
			registers.at(i)[2] = nr++; 
		}
		
		ROS_INFO("ACC: \n");
		divider = 100;
	}
	else if(opt == "MAG")
	{
		uint8_t nr = MAG_START_BYTE;

		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[2] = nr++;
		}
		
		ROS_INFO("MAG: \n");
		divider = 16;
	}
	else if(opt == "GYR")
	{
		uint8_t nr = GYR_START_BYTE;

		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[2] = nr++;
		}
		
		ROS_INFO("GYR: \n");
		divider = 16;
	}
	else if(opt == "EUL")
	{
		uint8_t nr = EUL_START_BYTE;

		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[2] = nr++;
		}
		
		ROS_INFO("EUL: \n");
		divider = 16;
	}
	else if(opt == "QUA")
	{
		uint8_t nr = QUA_START_BYTE;
		divider = 16384;

		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[2] = nr++;
		}
		
		ROS_INFO("QUA: \n");

		// w
		w_coord[1] = get_byte(read_DATA_W_LSB);
		w_coord[0] = get_byte(read_DATA_acc_msg.x = (float)convert_to_bytes(x_coord)/divider;
		acc_msg.y = (float)convert_to_bytes(y_coord)/divider;
		acc_msg.z = (float)convert_to_bytes(z_coord)/divider;
		W_MSB);
		
		ROS_INFO("w: %.4f\n",(float)convert_to_bytes(w_coord)/divider);
	}
	else if(opt == "LIA")
	{
		uint8_t nr = LIA_START_BYTE;

		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[2] = nr++;
		}
		
		ROS_INFO("LIA: \n");
		divider = 100;
	}
	else if(opt == "GRV")
	{
		uint8_t nr = GRV_START_BYTE;
		
		for(size_t i = 0; i < registers.size(); ++i)
		{
			registers.at(i)[2] = nr++;
		}
		
		ROS_INFO("GRV: \n");
		divider = 100;
	}
	else if(opt == "TEMP")
	{
		uint8_t temp_command[4] = {0xaa,0x1,0x34,0x1};
		uint8_t tmp = get_byte(temp_command);
		ROS_INFO("TEMP byte: %x\n\n", tmp);
		ROS_INFO("TEMP value: %d\n\n", tmp);
		return;
	}
 
	// x
	// msb
	x_coord[0] = get_byte(registers.at(1));
	ROS_INFO("x msb: %x",x_coord[0]);
	ros::Duration(0.01).sleep();

    // lsb
	x_coord[1] = get_byte(registers.at(0));
	ROS_INFO("x lsb: %x",x_coord[1]);
	ros::Duration(0.01).sleep();

	// ROS_INFO("x: %d\n",convert_to_bytes(x_coord));

	// y
	// msb
	y_coord[0] = get_byte(registers.at(3));
	ROS_INFO("y msb: %x",y_coord[0]);
	ros::Duration(0.01).sleep();

	// lsb
	y_coord[1] = get_byte(registers.at(2));
	ROS_INFO("y lsb: %x",y_coord[1]);
	ros::Duration(0.01).sleep();

	// ROS_INFO("y: %d\n",convert_to_bytes(y_coord));

	// z
	// msb
	z_coord[0] = get_byte(registers.at(5));
	ROS_INFO("z msb: %x",z_coord[0]);
	ros::Duration(0.01).sleep();

	// lsb
	z_coord[1] = get_byte(registers.at(4));
	ROS_INFO("z lsb: %x",z_coord[1]);
	ros::Duration(0.01).sleep();
	
	// ROS_INFO("z: %d\n\n",convert_to_bytes(z_coord));
	ROS_INFO(" (%.4f, %.4f, %.4f)\n",(float)convert_to_bytes(x_coord)/divider,(float)convert_to_bytes(y_coord)/divider,(float)convert_to_bytes(z_coord)/divider);

	// ROS_INFO(" (%.4f, %.4f, %.4f)\n",(float)convert_to_bytes(x_coord),(float)convert_to_bytes(y_coord),(float)convert_to_bytes(z_coord));

	communication->flush();


	// publish on topics
	if(opt == "ACC" && activate_acc_topic)
	{
		bno055_driver::acc acc_msg;
		acc_msg.x = (float)convert_to_bytes(x_coord) /divider;
		acc_msg.y = (float)convert_to_bytes(y_coord)/divider;
		acc_msg.z = (float)convert_to_bytes(z_coord)/divider;
		pub_acc.publish(acc_msg);
	}
	else if(opt == "MAG" && activate_mag_topic)
	{
		bno055_driver::mag mag_msg;
		mag_msg.x = (float)convert_to_bytes(x_coord) /divider;
		mag_msg.y = (float)convert_to_bytes(y_coord)/divider;
		mag_msg.z = (float)convert_to_bytes(z_coord)/divider;
		pub_mag.publish(mag_msg);
	}
	else if(opt == "GYR" && activate_gyr_topic)
	{
		bno055_driver::gyr gyr_msg;
		gyr_msg.x = (float)convert_to_bytes(x_coord) /divider;
		gyr_msg.y = (float)convert_to_bytes(y_coord)/divider;
		gyr_msg.z = (float)convert_to_bytes(z_coord)/divider;
		pub_gyr.publish(gyr_msg);
	}
	else if(opt == "EUL" && activate_eul_topic)
	{
		bno055_driver::eul eul_msg;
		eul_msg.x = (float)convert_to_bytes(x_coord) /divider;
		eul_msg.y = (float)convert_to_bytes(y_coord)/divider;
		eul_msg.z = (float)convert_to_bytes(z_coord)/divider;
		pub_eul.publish(eul_msg);
	}
	else if(opt == "QUA" && activate_qua_topic)
	{
		bno055_driver::qua qua_msg;
		qua_msg.x = (float)convert_to_bytes(x_coord) /divider;
		qua_msg.y = (float)convert_to_bytes(y_coord)/divider;
		qua_msg.z = (float)convert_to_bytes(z_coord)/divider;
		qua_msg.w = (float)convert_to_bytes(w_coord)/divider;
		pub_qua.publish(qua_msg);
		
	}
	else if(opt == "LIA" && activate_lia_topic)
	{
		bno055_driver::lia lia_msg;
		lia_msg.x = (float)convert_to_bytes(x_coord) /divider;
		lia_msg.y = (float)convert_to_bytes(y_coord)/divider;
		lia_msg.z = (float)convert_to_bytes(z_coord)/divider;
		pub_lia.publish(lia_msg);
	}
	else if(opt == "GRV" && activate_grv_topic)
	{
		bno055_driver::grv grv_msg;
		grv_msg.x = (float)convert_to_bytes(x_coord) /divider;
		grv_msg.y = (float)convert_to_bytes(y_coord)/divider;
		grv_msg.z = (float)convert_to_bytes(z_coord)/divider;
		pub_grv.publish(grv_msg);
	}
	
}

void BNO055_IMU::read_all_data_I2C(const string opt)
{
	int divider = 1;
	uint8_t x_coord[2] = {0x00};
	uint8_t y_coord[2] = {0x00};
	uint8_t z_coord[2] = {0x00};
	uint8_t w_coord[2] = {0x00}; // used only for quaternion: w

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
		w_coord[0] = communication->read_from_channel();

		uint8_t read_W_LSB_command[] = {0x20};
		communication->write_to_channel(read_W_LSB_command, 1);
		w_coord[1] = communication->read_from_channel();

		ROS_INFO("QUA: w = %.4f\n", (float)convert_to_bytes(w_coord)/divider);
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
		return;
	}

	send_command(registers.at(1)); //msb x
	x_coord[0] = communication->read_from_channel();

	send_command(registers.at(0)); //lsb x
	x_coord[1] = communication->read_from_channel();

	send_command(registers.at(3)); //msb y
	y_coord[0] = communication->read_from_channel();

	send_command(registers.at(2)); //lsb y
	y_coord[1] = communication->read_from_channel();

	send_command(registers.at(5)); //msb z
	z_coord[0] = communication->read_from_channel();

	if(z_coord[0] == 0x80 || z_coord[0] == 0x83)
	{
		z_coord[0] = 0x00;
	}

	send_command(registers.at(4)); //lsb z
	z_coord[1] = communication->read_from_channel();

	ROS_INFO("%x, %x, %x, %x, %x, %x",x_coord[1], x_coord[0], y_coord[1], y_coord[0], z_coord[1], z_coord[0]);
	ROS_INFO(" (%.4f, %.4f, %.4f)\n",(float)convert_to_bytes(x_coord)/divider,(float)convert_to_bytes(y_coord)/divider,(float)convert_to_bytes(z_coord)/divider);

	// publish on topics
	if(opt == "ACC" && activate_acc_topic)
	{
		bno055_driver::acc acc_msg;
		acc_msg.x = (float)convert_to_bytes(x_coord)/divider;
		acc_msg.y = (float)convert_to_bytes(y_coord)/divider;
		acc_msg.z = (float)convert_to_bytes(z_coord)/divider;
		pub_acc.publish(acc_msg);
		// acc_msg = create_message_bno(x_coord,y_coord,z_coord,w_coord,divider);
		// publish_message_bno(pub_acc,acc_msg);
	}
	else if(opt == "MAG" && activate_mag_topic)
	{
		bno055_driver::mag mag_msg;
		mag_msg.x = (float)convert_to_bytes(x_coord) /divider;
		mag_msg.y = (float)convert_to_bytes(y_coord)/divider;
		mag_msg.z = (float)convert_to_byteacc_msg.x = (float)convert_to_bytes(x_coord)/divider;
		acc_msg.y = (float)convert_to_bytes(y_coord)/divider;
		acc_msg.z = (float)convert_to_bytes(z_coord)/divider;
		s(z_coord)/divider;
		pub_mag.publish(mag_msg);
		// mag_msg = create_message_bno(x_coord,y_coord,z_coord,w_coord,divider);
		// publish_message_bno(pub_mag,mag_msg);
	}
	else if(opt == "GYR" && activate_gyr_topic)
	{
		bno055_driver::gyr gyr_msg;
		gyr_msg.x = (float)convert_to_bytes(x_coord) /divider;
		gyr_msg.y = (float)convert_to_bytes(y_coord)/divider;
		gyr_msg.z = (float)convert_to_bytes(z_coord)/divider;
		pub_gyr.publish(gyr_msg);
		// gyr_msg = create_message_bno(x_coord,y_coord,z_coord,w_coord,divider);
		// publish_message_bno(pub_gyr,gyr_msg);
		
	}
	else if(opt == "EUL" && activate_eul_topic)
	{
		bno055_driver::eul eul_msg;
		eul_msg.x = (float)convert_to_bytes(x_coord) /divider;
		eul_msg.y = (float)convert_to_bytes(y_coord)/divider;
		eul_msg.z = (float)convert_to_bytes(z_coord)/divider;
		pub_eul.publish(eul_msg);
		// eul_msg = create_message_bno(x_coord,y_coord,z_coord,w_coord,divider);
		// publish_message_bno(pub_eul,eul_msg);
	}
	else if(opt == "QUA" && activate_qua_topic)
	{
		bno055_driver::qua qua_msg;
		qua_msg.x = (float)convert_to_bytes(x_coord) /divider;
		qua_msg.y = (float)convert_to_bytes(y_coord)/divider;
		qua_msg.z = (float)convert_to_bytes(z_coord)/divider;
		qua_msg.w = (float)convert_to_bytes(w_coord)/divider;
		pub_qua.publish(qua_msg);
		// qua_msg = create_message_bno(x_coord,y_coord,z_coord,w_coord,divider);
		// publish_message_bno(pub_qua,qua_msg);
		
	}
	else if(opt == "LIA" && activate_lia_topic)
	{
		bno055_driver::lia lia_msg;
		lia_msg.x = (float)convert_to_bytes(x_coord) /divider;
		lia_msg.y = (float)convert_to_bytes(y_coord)/divider;
		lia_msg.z = (float)convert_to_bytes(z_coord)/divider;
		pub_lia.publish(lia_msg);
		// lia_msg = create_message_bno(x_coord,y_coord,z_coord,w_coord,divider);
		// publish_message_bno(pub_lia,lia_msg);
	}
	else if(opt == "GRV" && activate_grv_topic)
	{
		bno055_driver::grv grv_msg;
		grv_msg.x = (float)convert_to_bytes(x_coord) /divider;
		grv_msg.y = (float)convert_to_bytes(y_coord)/divider;
		grv_msg.z = (float)convert_to_bytes(z_coord)/divider;
		pub_grv.publish(grv_msg);
		// grv_msg = create_message_bno(x_coord,y_coord,z_coord,w_coord,divider);
		// publish_message_bno(pub_grv,grv_msg);
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
	communication->write_to_channel(command,sizeof(command));
}

void BNO055_IMU::publish_message()
{

}

void BNO055_IMU::create_message()
{

}

// template<class T, class U>
// void BNO055_IMU::publish_message_bno(T publisher, U message)
// {
// 	publisher.publish(message);
// }

// template<class U>
// U BNO055_IMU::create_message_bno(uint8_t x_coord[2],uint8_t y_coord[2],uint8_t z_coord[2],uint8_t w_coord[2],int divider)
// {
// 	U message;
// 	message.x = (float)convert_to_bytes(x_coord) /divider;
// 	message.y = (float)convert_to_bytes(y_coord)/divider;
// 	message.z = (float)convert_to_bytes(z_coord)/divider;
// 	if(std::is_same<U,bno055_driver::qua>::value)
// 	{
// 		message.w = (float)convert_to_bytes(w_coord)/divider;
// 	}
// 	return message;
// }

void BNO055_IMU::activate_topics(ros::NodeHandle* nh)
{
	nh->getParam("activate_acc_topic", activate_acc_topic);
	nh->getParam("activate_mag_topic", activate_mag_topic);
	nh->getParam("activate_gyr_topic", activate_gyr_topic);
	nh->getParam("activate_eul_topic", activate_eul_topic);
	nh->getParam("activate_qua_topic", activate_qua_topic);
	nh->getParam("activate_lia_topic", activate_lia_topic);
	nh->getParam("activate_grv_topic", activate_grv_topic);
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"bno055_node");
	ros::NodeHandle nh;
	ros::Rate rate(10);
	int comm_type = -1;
	CommunicationFactory cf = CommunicationFactory();
	nh.getParam("communication_type",comm_type);
	ROS_INFO("communication_type: %d\n",comm_type);

	switch(comm_type)
	{
		case 0:
		{
			int baudrate = 0;
			string uart_port_name;
			nh.getParam("baudrate",baudrate);
			nh.getParam("uart_port_name",uart_port_name);
			Communication* c = cf.create_communication("UART_115200", (char*)uart_port_name.c_str(),0);
			BNO055_IMU bno = BNO055_IMU(&nh,c);

			bno.activate_topics(&nh);

			bno.stop_communication();
			ros::Duration(0.1).sleep();

			bno.start_communication();
			ros::Duration(0.1).sleep();

			uint8_t OPR_MODE_COMMAND[5] = {0xaa, 0x0, 0x3d, 0x1, 0x9}; // Compass
			bno.get_communication()->write_to_channel(OPR_MODE_COMMAND,5);
			ros::Duration(0.1).sleep();
	
			uint8_t bte1 = bno.get_communication()->read_from_channel();
			uint8_t bte2 = bno.get_communication()->read_from_channel();
			
			ROS_INFO("\n\n%x ; %x\n\n", bte1, bte2);
			ros::Duration(0.1).sleep();

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
			int address = 0;
			string i2c_port_name;
			nh.getParam("address",address);
			nh.getParam("i2c_port_name",i2c_port_name);

			Communication* c = cf.create_communication("I2C", (char*)i2c_port_name.c_str(), address);
			BNO055_IMU bno = BNO055_IMU(&nh,c);
			
			bno.activate_topics(&nh);

			bno.start_communication();
			ros::Duration(1).sleep();
			
			uint8_t write_amg_cmd[2] = {0x3d, 0x08}; // IMU
			bno.get_communication()->write_to_channel(write_amg_cmd,2);
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
