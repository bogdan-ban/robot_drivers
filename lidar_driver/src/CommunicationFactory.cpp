#include "lidar_driver/CommunicationFactory.h"

CommunicationFactory::CommunicationFactory()
{

}

Communication* CommunicationFactory::create_communication(string name, char* path, int baudrate_or_addr)
{
	if(name == "UART")
	{
		return new UartCommunication(path, baudrate_or_addr); 
	}
	else if(name == "I2C")
	{
		return new I2cCommunication(path, baudrate_or_addr); 
	}
	else
	{
		return nullptr;
	}
}
