#include <string>
#include <iostream>
#include <cstdio>
#include <windows.h>
//#include <math.h>
#include "serial/serial.h"
#include <thread>

#define SHORTSIZE 2
#define MESSAGESIZE SHORTSIZE+1
#define MESSAGESIZE_WRITE MESSAGESIZE
#define LOOPRATE_MS 2
#define LOOPCOUNTS_INT 500
#define POWER_OFF 32768.0
#define BACKWARD_MAX 0
#define FORWARD_MAX USHRT_MAX
#define MESSAGE_MAX 65535.0
#define RPM_MAX 10000.0
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

typedef union {
	uint16_t unsignedShort;
	uint8_t binary[SHORTSIZE];
} binaryUShort;


//void my_sleep(unsigned long milliseconds) {
//	Sleep(milliseconds); // 100 ms
//}

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

binaryUShort sentNumber;

binaryUShort receivedNumber;

uint8_t incomingData[1030];

uint8_t endMessage = 0x0A;

void serial_write_trigger(bool* readyToWrite)
{
	std::chrono::milliseconds duraWrite(LOOPRATE_MS);
	while (true)
	{
		*readyToWrite = true;
		std::this_thread::sleep_for(duraWrite);
	}
}



//void serial_read_thread(serial::Serial* mySerialPtr)
//{
//	std::chrono::milliseconds dura(1000);
//
//	std::this_thread::sleep_for(dura);
//	size_t whatIsAvailable;
//	size_t bytes_read;
//	unsigned int readCount;
//	readCount = 0;
//
//	while(true){
//		cout << "Bytes Available: " << mySerialPtr->available() << endl;
//		
//		if (mySerialPtr->waitReadable())
//		{
//			string result = mySerialPtr->read(1);
//		}
//		std::this_thread::sleep_for(dura);
//
//	}	
//}

uint16_t convertSpeedToMessage(float rpm)
{
	float rpmScaled = rpm / RPM_MAX;
	return (uint16_t) (rpmScaled * (MESSAGE_MAX - POWER_OFF) + POWER_OFF);
}

int main()
{
	string port = "COM5";
	unsigned long baud = 115200;

	// port, baudrate, timeout in milliseconds
	serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));
	my_serial.setTimeout(serial::Timeout::max(), 1, 0, 1, 0);

	my_serial.flush();
	Sleep(100);

	cout << "Is the serial port open?";
	if (my_serial.isOpen())
		cout << " Yes." << endl;
	else
		cout << " No." << endl;

				// don't forget to pre-allocate memory


	
	//string test_string = "Suppiluluima: the king of the Hittites";

	//char newLine[] = "\n";
	
	// uint8_t endMessage = 0;
	// endMessage = (uint8_t) newLine;

//	cout << endMessage;

	//std::thread threadObj(serial_read_thread, &my_serial);

	//if (threadObj.joinable())
	//{
	//	//threadObj.join();
	//	//std::cout << "Joined Thread " << std::endl;
	//	std::cout << "Detaching Thread " << std::endl;
	//	threadObj.detach();
	//}

	bool readyToWrite;

	std::thread threadWrite(serial_write_trigger, &readyToWrite);
	if (threadWrite.joinable())
	{
		std::cout << "Detaching Thread " << std::endl;
		threadWrite.detach();
	}
	// Test the timeout, there should be 1 second between prints
	//cout << "Timeout == 1000ms, asking for exactly what was written." << endl;

	//cout << "Timeout == 10ms, asking for exactly what was written." << endl;

	std::chrono::milliseconds duraWrite(LOOPRATE_MS);
	//my_serial.flushOutput();
	
	uint8_t message[MESSAGESIZE_WRITE];
	message[MESSAGESIZE_WRITE-1] = endMessage;

	sentNumber.unsignedShort = 0;
	binaryUShort sentNumberLast = sentNumber;
	int writeCount = 0;
	int writeCountOld = writeCount;
	size_t bytes_wrote;
	size_t whatIsAvailable;
	size_t bytes_read = 0;
	unsigned int readCount;
	readCount = 0;
	 
	while (writeCount < 1000000) {
		
		if (readyToWrite)
		{
			/*size_t bytes_wrote = my_serial.write(test_string);

			string result = my_serial.read(test_string.length());*/

			sentNumber.unsignedShort = convertSpeedToMessage(1000); //36045; // / 30.0 * 2.0 * 3.141));
			
			std::memcpy(message, sentNumber.binary, SHORTSIZE);
			bytes_wrote = my_serial.write(message, MESSAGESIZE_WRITE);
			//my_serial.write(&endMessage, 1);

			//size_t theLength= my_serial.read(incomingData, FLOATSIZE);

			//for (int i = 1; i < FLOATSIZE; i++)
				//receivedNumber.binary[i] = incomingData[i];

			writeCount += 1;
			//modRes = writeCount % LOOPCOUNTS;
			
			if (writeCount % LOOPCOUNTS_INT == 0)
			{
				//cout << "Writ Iter: " << writeCount << ", Len: " << bytes_wrote << ", Val: " << sentNumber.floatingPoint << " BIN: " << (int)sentNumber.binary[0] << " " << (int)sentNumber.binary[1] << " " << (int)sentNumber.binary[2] << " " << (int)sentNumber.binary[3] << endl;
				printf("Delta: %d, lastSent: %d, received: %d \n", sentNumberLast.unsignedShort - receivedNumber.unsignedShort,  sentNumberLast.unsignedShort, receivedNumber.unsignedShort);

				//writeCountOld = writeCount;
			}
			if (writeCount % LOOPCOUNTS_INT == 1)
			{
				//cout << "Read Iter: " << readCount << ", Len: " << bytes_read << ", Val: " << receivedNumber.floatingPoint << " BIN: " << (int)receivedNumber.binary[0] << " " << (int)receivedNumber.binary[1] << " " << (int)receivedNumber.binary[2] << " " << (int)receivedNumber.binary[3] << endl;
				//cout << "-----------------------------------------------------------------------------" << endl;

				//	writeCountOld = writeCount;
			}
			
			sentNumberLast = sentNumber;
			//if (writeCount == writeCountOld + 1 )
			//{
			//}
			//std::this_thread::sleep_for(duraWrite);
			readyToWrite = false;
		}
		
		whatIsAvailable = my_serial.available();

		//cout << "Bytes Available: " << whatIsAvailable << end;l

		if (whatIsAvailable > MESSAGESIZE - 1)
		{
			if(whatIsAvailable > MESSAGESIZE)
			{ 
				cout << "Bytes Available: " << whatIsAvailable << endl;
			}
			bytes_read = my_serial.read(incomingData, whatIsAvailable);
			//cout << "Bytes read: " << length << endl;
			if (bytes_read == MESSAGESIZE && incomingData[MESSAGESIZE - 1] == endMessage)
			{
				// parse the data to the shared float
				std::memcpy(receivedNumber.binary, incomingData, SHORTSIZE);
				//std::memcpy(otherNumber.binary, &(incomingData[FLOATSIZE]), FLOATSIZE);
				readCount++;
				//if (readCount % LOOPCOUNTS_INT == 0)
				//{
				//	cout << "Read Iter: " << readCount << ", Len: " << bytes_read << ", Val: " << receivedNumber.floatingPoint << endl;
				//}

			}
		}

	}

	return 0;
}