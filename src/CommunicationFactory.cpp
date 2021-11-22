#include "robot_drivers/CommunicationFactory.h"

CommunicationFactory::CommunicationFactory()
{

}

Communication* CommunicationFactory::create_communication(string name)
{
	if(name == "UART")
	{
		// UartCommunication uart = UartCommunication("/dev/ttyS0",115200);
		// return &uart;

		return new UartCommunication("/dev/ttyS0",115200);

	}else if(name == "I2C")
	{
		// I2cCommunication i2c = I2cCommunication("/dev/i2c-2", 0x50);
		// return &i2c;

		return new I2cCommunication("/dev/i2c-2", 0x50);

	}else
	{
		return nullptr;
	}
}