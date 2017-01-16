#ifndef SERIAL_COMMUNICATOR_H
#define SERIAL_COMMUNICATOR_H
#pragma

#include "serial/serial.h"

typedef union {
    uint16_t unsignedShort;
    uint8_t binary[SHORTSIZE];
} binaryUShort;

class SerialCommunicator
{
  public:
    SerialCommunicator();
    ~SerialCommunicator();
    void enumerate_ports();
    void print_usage();

  private:
    string port;
    unsigned long baud;
    // port, baudrate, timeout in milliseconds
    serial::Serial my_serial;

    binaryUShort sentNumber;
    binaryUShort receivedNumber;

    uint8_t incomingData[1030];
    uint8_t endMessage = 0x0A;



}

#endif
