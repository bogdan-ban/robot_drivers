#include <iostream>
#include "lidar_driver/communication.h"
#include "lidar_driver/uart_communication.h"
#include "lidar_driver/i2c_communication.h"

using namespace std;


class CommunicationFactory
{

public:

	CommunicationFactory();

	Communication* create_communication(string name, char* path, int baudrate_or_addr);

};
