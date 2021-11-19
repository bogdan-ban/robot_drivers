#include <stdio.h>
#include "communication.h"

class Sensor
{
public:
	Sensor(Communication* _communication): communication(_communication) {}

	virtual void start_communication() = 0;
	virtual void stop_communication() = 0;
	virtual uint8_t* read_data(int size) = 0;
	virtual void send_command(uint8_t* command) = 0;
	virtual void publish_message() = 0;
	virtual void create_message() = 0;

	~Sensor() {}

private:
	Communication* communication;
	void* message;
};
