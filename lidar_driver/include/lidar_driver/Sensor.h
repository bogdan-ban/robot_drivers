#include <iostream>
#include <string>

class Sensor
{
protected:
	std::string communication_channel;

public:
	Sensor() : communication_channel("")
	{
	}

	Sensor(std::string new_communication_channel) : communication_channel(new_communication_channel)
	{
	}

	virtual void change_path(std::string new_comunication_channel) = 0;
};
