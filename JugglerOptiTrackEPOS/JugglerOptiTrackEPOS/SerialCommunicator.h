#ifndef SERIAL_COMMUNICATOR_H
#define SERIAL_COMMUNICATOR_H

#include "serial/serial.h"
#include <mutex>

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
	void readThread();

  private:
    string port;
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
	
	sentNumber.unsignedShort;
	binaryUShort sentNumberLast;
	int writeCount;
	size_t bytes_wrote;
	size_t whatIsAvailable;
	size_t bytes_read;
	unsigned int readCount;
	mutex mutexRec;
	std::chrono::milliseconds durationReadThreadSleep(LOOPRATE_MS);

}

#endif
