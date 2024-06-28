/*!
 * \file serialPort.hpp
 *
 * Opens, reads from, and writes to a serial port
 *
 *  Created on: Mar 15, 2013
 *      Author: Buddy Michini
 *      Maintainer: Brett Lopez
 *
 */

#ifndef SERIALPORT_H_
#define SERIALPORT_H_

#include <arpa/inet.h>
#include <fcntl.h>
#include <iostream>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <math.h>
#include <semaphore.h>
#include <signal.h>
#include <sys/select.h>

/// Maximum serial port buffer size
#define SER_BUF_SZ 5000

namespace libserial {
class SerialPort {

  int serial;
  char buffer[SER_BUF_SZ];

public:
  SerialPort();

  bool initialized;
  int spInitialize(std::string usbport, int baudrate);
  int spSend(char *pkt);
  int spSend(char *pkt, int sz);
  int spSend(const char *pkt, int sz);
  int spSend(uint8_t *pkt, uint8_t sz);
  char *spReceive();
  char *spReceive(int nbytes);
  char spReceiveSingle();
  void spClose();
};
}; // namespace libserial

#endif
