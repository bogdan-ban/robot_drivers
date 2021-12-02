#include <iostream>
#include "communication.h"
#include "uart_communication.h"
#include "i2c_communication.h"

using namespace std;


class CommunicationFactory
{

public:

	CommunicationFactory();

	Communication* create_communication(string name, char* path, int baudrate_or_addr);

};
