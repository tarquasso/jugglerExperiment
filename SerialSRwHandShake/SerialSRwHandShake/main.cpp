#include <string>
#include <iostream>
#include <cstdio>
#include <windows.h>

#include "serial/serial.h"

#define FLOATSIZE 4

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

typedef union {
	float floatingPoint;
	uint8_t binary[FLOATSIZE];
} binaryFloat;



void my_sleep(unsigned long milliseconds) {
	Sleep(milliseconds); // 100 ms
}

void enumerate_ports()
{
	vector<serial::PortInfo> devices_found = serial::list_ports();

	vector<serial::PortInfo>::iterator iter = devices_found.begin();

	while (iter != devices_found.end())
	{
		serial::PortInfo device = *iter++;

		printf("(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
			device.hardware_id.c_str());
	}
}

void print_usage()
{
	cerr << "Usage: test_serial {-e|<serial port address>} ";
	cerr << "<baudrate> [test string]" << endl;
}

int main()
{
	string port = "COM5";
	unsigned long baud = 9600;

	// port, baudrate, timeout in milliseconds
	serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));

	cout << "Is the serial port open?";
	if (my_serial.isOpen())
		cout << " Yes." << endl;
	else
		cout << " No." << endl;

	uint8_t incomingData[FLOATSIZE];			// don't forget to pre-allocate memory

	binaryFloat sentNumber;
	sentNumber.floatingPoint = 12.41f;

	binaryFloat receivedNumber;

	int count = 0;
	string test_string = "Suppiluluima: the king of the Hittites";

	char newLine[] = "\n";
	uint8_t endMessage = 0x0A;

	// uint8_t endMessage = 0;
	// endMessage = (uint8_t) newLine;

	cout << endMessage;

	// Test the timeout, there should be 1 second between prints
	cout << "Timeout == 1000ms, asking for exactly what was written." << endl;
	my_serial.setTimeout(serial::Timeout::max(), 1, 0, 1, 0);
	cout << "Timeout == 10ms, asking for exactly what was written." << endl;
	while (count < 1000000) {
		/*size_t bytes_wrote = my_serial.write(test_string);

		string result = my_serial.read(test_string.length());*/

		size_t bytes_wrote = my_serial.write(sentNumber.binary, FLOATSIZE);
		my_serial.write(&endMessage, 1);

		size_t theLength= my_serial.read(incomingData, FLOATSIZE);

		for (int i = 1; i < FLOATSIZE; i++)
			receivedNumber.binary[i] = incomingData[i];


		cout << "Iteration: " << count << ", Bytes written: ";
		cout << bytes_wrote << ", What is sent: " << sentNumber.floatingPoint << ", Bytes read: ";
		cout << theLength << ", What is	read: " << receivedNumber.floatingPoint << endl;

		count += 1;
	}

	return 0;
}