#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <iostream>
#include <unistd.h>

using namespace std;


class Communication
{
public:

	virtual int start() = 0;

	virtual uint8_t read_from_channel() = 0;

	virtual void write_to_channel(const uint8_t buffer[], int size) = 0;

	virtual void stop() = 0;

	// virtual int convert_to_bytes(uint8_t vec[2]) = 0;

	// virtual void read_all_data(const string opt) = 0;

};

#endif