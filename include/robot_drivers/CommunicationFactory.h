#include <iostream>
#include "robot_drivers/communication.h"
#include "robot_drivers/uart_communication.h"
#include "robot_drivers/i2c_communication.h"

using namespace std;


class CommunicationFactory
{

public:

	CommunicationFactory();

	Communication* create_communication(string name);

};