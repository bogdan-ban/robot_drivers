#include <stdio.h>
#include "communication.h"


class Sensor
{
public:
	Sensor(Communication* _communication): communication(_communication) {}

	virtual void start_communication() = 0;
	virtual void stop_communication() = 0;
	virtual void read_data(uint8_t* buffer, int size) = 0;
	virtual void send_command(uint8_t* command) = 0;
	virtual void publish_message() = 0;
	virtual void create_message() = 0;

	~Sensor() {}

protected:
	Communication* communication;
	void* message;
};