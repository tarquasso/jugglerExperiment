#include "SerialCommunicator.h"

#include <iostream>

//#include <cstdio>
//#include <windows.h>
//#include <math.h>
//#include <thread>

#define LOOPRATE_MS 2
#define LOOPCOUNTS_INT 500
#define POWER_OFF 32768.0f
#define BACKWARD_MAX 0
#define FORWARD_MAX USHRT_MAX
#define MESSAGE_MAX 65535.0f
#define RPM_MAX 10000.0f
//using std::string;
//using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;


/*
Constructor
*/
SerialCommunicator::SerialCommunicator():
    port("COM5"),
    baud(115200),
    my_serial(port, baud, serial::Timeout::simpleTimeout(1))
{
    //my_serial.setTimeout(serial::Timeout::max(), 1, 0, 1, 0);
    
	my_serial.flush(); //flush the serial line

    //	Sleep(100);
    cout << "Is the serial port open?";
	if (my_serial.isOpen())
		cout << " Yes." << endl;
	else
		cout << " No." << endl;

	// setup the message by setting last byte to be endmessage
	message[MESSAGESIZE_WRITE - 1] = endMessage;

	sentNumber.unsignedShort = 0;
	//sentNumberLast = sentNumber;
	writeCount = 0;
	//int writeCountOld = writeCount;
	bytes_read = 0;
	readCount = 0;
	//durationReadThreadSleepUS = 500;
}

/*
Destructor
*/

SerialCommunicator::~SerialCommunicator()
{

}

/*
void serial_write_trigger(bool* readyToWrite)
{
	std::chrono::milliseconds duraWrite(LOOPRATE_MS);
	while (true)
	{
		*readyToWrite = true;
		std::this_thread::sleep_for(duraWrite);
	}
}
*/

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

void SerialCommunicator::sendMotorRPM(float rpm)
{
	sentNumber.unsignedShort = convertSpeedRPMToMessage(rpm); //36045; // / 30.0 * 2.0 * 3.141));

	std::memcpy(message, sentNumber.binary, SHORTSIZE);
	bytes_wrote = my_serial.write(message, MESSAGESIZE_WRITE);
	
	writeCount += 1;
	/*
	if (writeCount % LOOPCOUNTS_INT == 0)
	{
		//cout << "Writ Iter: " << writeCount << ", Len: " << bytes_wrote << ", Val: " << sentNumber.floatingPoint << " BIN: " << (int)sentNumber.binary[0] << " " << (int)sentNumber.binary[1] << " " << (int)sentNumber.binary[2] << " " << (int)sentNumber.binary[3] << endl;
		printf("Delta: %d, lastSent: %d, received: %d \n", sentNumberLast.unsignedShort - receivedNumber.unsignedShort, sentNumberLast.unsignedShort, receivedNumber.unsignedShort);
		//writeCountOld = writeCount;
	}
	if (writeCount % LOOPCOUNTS_INT == 1)
	{
		//cout << "Read Iter: " << readCount << ", Len: " << bytes_read << ", Val: " << receivedNumber.floatingPoint << " BIN: " << (int)receivedNumber.binary[0] << " " << (int)receivedNumber.binary[1] << " " << (int)receivedNumber.binary[2] << " " << (int)receivedNumber.binary[3] << endl;
		//cout << "-----------------------------------------------------------------------------" << endl;

		//	writeCountOld = writeCount;
	}
	sentNumberLast = sentNumber;
	*/
}

float SerialCommunicator::readMotorRPM()
{
	//mutexRec.lock();
	//uint16_t receivedVal = receivedNumber.unsignedShort;
	//mutexRec.unlock();

	whatIsAvailable = my_serial.available();
	
	//cout << "Bytes Available: " << whatIsAvailable << end;l
	if (whatIsAvailable > MESSAGESIZE - 1)
	{
		if (whatIsAvailable > MESSAGESIZE)
		{
			cout << "Bytes Available: " << whatIsAvailable << endl;
		}
		bytes_read = my_serial.read(incomingData, whatIsAvailable);
		//cout << "Bytes read: " << length << endl;
		//if (bytes_read == MESSAGESIZE && incomingData[MESSAGESIZE - 1] == endMessage)
		if (incomingData[bytes_read - 1] == endMessage)
		{
			// parse the data to the shared float
			//mutexRec.lock();
			std::memcpy(receivedNumber.binary, &(incomingData[bytes_read - 1 - SHORTSIZE]), SHORTSIZE);
			//std::memcpy(receivedNumber.binary, incomingData), SHORTSIZE);

			//mutexRec.unlock();

			//std::memcpy(otherNumber.binary, &(incomingData[FLOATSIZE]), FLOATSIZE);
			readCount++;
			//if (readCount % LOOPCOUNTS_INT == 0)
			//{
			//	cout << "Read Iter: " << readCount << ", Len: " << bytes_read << ", Val: " << receivedNumber.floatingPoint << endl;
			//}
		}
	}

	// return what is there already
	return this->convertMessageToSpeedRPM(receivedNumber.unsignedShort);
}

/*
int SerialCommunicator::main()
{
	
	//bool readyToWrite;

	//std::thread threadWrite(serial_write_trigger, &readyToWrite);
	//if (threadWrite.joinable())
	//{
	//	std::cout << "Detaching Thread " << std::endl;
	//	threadWrite.detach();
	//}
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
			// size_t bytes_wrote = my_serial.write(test_string);

			// string result = my_serial.read(test_string.length());

			sentNumber.unsignedShort = convertSpeedToMessage(1000); //36045; // / 30.0 * 2.0 * 3.141));
			
			std::memcpy(message, sentNumber.binary, SHORTSIZE);
			bytes_wrote = my_serial.write(message, MESSAGESIZE_WRITE);
			//my_serial.write(&endMessage, 1);

			//size_t theLength= my_serial.read(incomingData, FLOATSIZE);

			//for (int i = 1; i < FLOATSIZE; i++)
				//receivedNumber.binary[i] = incomingData[i];

			
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
*/


uint16_t SerialCommunicator::convertSpeedRPMToMessage(float rpm)
{
	float rpmScaled = rpm / RPM_MAX;
	return (uint16_t)(rpmScaled * (MESSAGE_MAX - POWER_OFF) + POWER_OFF);
}

float SerialCommunicator::convertMessageToSpeedRPM(uint16_t message)
{
	float msgShifted = ((float)message - POWER_OFF);
	float rpmNormalized = msgShifted / (MESSAGE_MAX - POWER_OFF);
	return (rpmNormalized * RPM_MAX);
}

void SerialCommunicator::enumerate_ports()
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

void SerialCommunicator::print_usage()
{
	cerr << "Usage: test_serial {-e|<serial port address>} ";
	cerr << "<baudrate> [test string]" << endl;
}
