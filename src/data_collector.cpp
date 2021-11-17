#include "robot_drivers/data_collector.h"


DataCollector::DataCollector(ros::NodeHandle *nh)
{
	
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"data_collector_node");

	ros::NodeHandle nh;

	ros::Rate rate(10);

	DataCollector dc = DataCollector(&nh);	

	int option = 0;
	cout << "Choose communication: (UART)0, (I2C)1, (SPI)2\n";
	cin >> option;

	switch(option)
	{
		case 0:
		{
			/** UartCommunication uart = UartCommunication("/dev/ttyS0",115200);
			dc.communication = &uart;

			dc.communication->start();
			ros::Duration(1).sleep();

			const uint8_t OPR_MODE_COMMAND[5] = {0xaa, 0x00, 0x3d, 0x1, 0x7}; // AMG
			dc.communication->write_to_channel(OPR_MODE_COMMAND,5);
			cout << dc.communication->read_from_channel() << endl;
			ros::Duration(1).sleep();

			while(ros::ok())
			{
				dc.communication->read_all_data("ACC");
				dc.communication->read_all_data("MAG");
				dc.communication->read_all_data("GYR");
				dc.communication->read_all_data("EUL");
				dc.communication->read_all_data("QUA");
				dc.communication->read_all_data("LIA");
				dc.communication->read_all_data("GRV");
				ros::Duration(1).sleep();
			}

			dc.communication->stop();
			*/

			dc.uc = new UartCommunication("/dev/ttyS0",115200);
			dc.uc->start();
			ros::Duration(1).sleep();

			const uint8_t OPR_MODE_COMMAND[5] = {0xaa, 0x00, 0x3d, 0x1, 0x7}; // AMG
			dc.uc->write_to_channel(OPR_MODE_COMMAND,5);
			cout << dc.uc->read_from_channel() << endl;
			ros::Duration(1).sleep();

			while(ros::ok())
			{
				dc.uc->read_all_data("ACC");
				dc.uc->read_all_data("MAG");
				dc.uc->read_all_data("GYR");
				dc.uc->read_all_data("EUL");
				dc.uc->read_all_data("QUA");
				dc.uc->read_all_data("LIA");
				dc.uc->read_all_data("GRV");
				ros::Duration(1).sleep();
			}

			dc.uc->stop();

		}
		break;

		case 1:
		{
			// i2c code

			dc.i2c = new I2cCommunication("/dev/i2c-2",0x57);

			dc.i2c->start();
			ros::Duration(1).sleep();

			while(ros::ok())
			{
				dc.i2c->read_all_data("ACC");
				dc.i2c->read_all_data("MAG");
				dc.i2c->read_all_data("GYR");
				dc.i2c->read_all_data("EUL");
				dc.i2c->read_all_data("QUA");
				dc.i2c->read_all_data("LIA");
				dc.i2c->read_all_data("GRV");
				ros::Duration(1).sleep();
			}

			dc.i2c->stop();

		}
		break;

		default:
		cout << "Please choose an option from [0,1] interval";
	}

	ros::spin();

	return 0;
}
