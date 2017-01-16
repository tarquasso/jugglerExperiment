#ifndef SERIAL_COMMUNICATOR_H
#define SERIAL_COMMUNICATOR_H

#include <string>
#include "serial/serial.h"

using std::string;

//#include <mutex>

#define SHORTSIZE 2
#define MESSAGESIZE SHORTSIZE+1
#define MESSAGESIZE_WRITE MESSAGESIZE

typedef union {
    uint16_t unsignedShort;
    uint8_t binary[SHORTSIZE];
} binaryUShort;

class SerialCommunicator
{
  public:
    SerialCommunicator();
    ~SerialCommunicator();
	
	void sendMotorRPM(float rpm);
	float readMotorRPM();

    void enumerate_ports();
    void print_usage();

	static uint16_t convertSpeedRPMToMessage(float rpm);
	static float convertMessageToSpeedRPM(uint16_t message);

protected:
	//void readThread();

  private:
    std::string port;
    unsigned long baud;
    // port, baudrate, timeout in milliseconds
    serial::Serial my_serial;

    binaryUShort sentNumber;
    binaryUShort receivedNumber;

    uint8_t incomingData[1030];
    uint8_t endMessage = 0x0A;

	//std::chrono::milliseconds duraWrite(LOOPRATE_MS);
	//my_serial.flushOutput();

	uint8_t message[MESSAGESIZE_WRITE];
	binaryUShort sentNumberLast;
	uint64_t writeCount;
	size_t bytes_wrote;
	size_t whatIsAvailable;
	size_t bytes_read;
	uint64_t readCount;
	//mutex mutexRec;
	//std::chrono::microseconds durationReadThreadSleepUS;

};

#endif
