/*!
 * \file serialPort.cpp
 *
 * Opens, reads from, and writes to a serial port
 *
 *  Created on: Mar 15, 2013
 *      Author: Buddy Michini
 *      Maintainer: Brett Lopez
 *
 */

#include "serial.h"

namespace libserial {
SerialPort::SerialPort() {
  initialized = false;
  serial = 0;
}

void signal_handler_IO(int status);

/**
 * Open a serial port on your computer.
 * @param hwDevice Address of your serial port (ex: /dev/ttyUSB0)
 * @param baudRate Serial port baud rate (ex: 57600)
 * @return 1 if port opened, 0 otherwise
 */
int SerialPort::spInitialize(std::string hwDevice, int baudRate) {

  struct termios attr;
  struct sigaction saio;

  if ((serial = open(hwDevice.c_str(), O_RDWR | O_EXCL)) < 0) {
    printf("Unable to open serial port %s \n", hwDevice.c_str());
    return 0;
  }

  if (tcgetattr(serial, &attr) < 0) {
    printf("Call to tcgetattr failed \n");
    return 0;
  }

  attr.c_iflag = 0;
  attr.c_oflag = 0;
  attr.c_cflag = CLOCAL | CREAD | CS8;
  attr.c_lflag = 0;
  attr.c_cc[VTIME] = 0; // timeout in tenths of a second
  attr.c_cc[VMIN] = 1;  // Only wait for a single char

  if (baudRate == 38400) { // ghetto hack
    cfsetispeed(&attr, B38400);
    cfsetospeed(&attr, B38400);
  } else if (baudRate == 57600) {
    cfsetispeed(&attr, B57600);
    cfsetospeed(&attr, B57600);
  } else if (baudRate == 115200) {
    cfsetispeed(&attr, B115200);
    cfsetospeed(&attr, B115200);
  } else if (baudRate == 19200) {
    cfsetispeed(&attr, B19200);
    cfsetospeed(&attr, B19200);
  } else if (baudRate == 460800) {
    cfsetispeed(&attr, B460800);
    cfsetospeed(&attr, B460800);
  } else if (baudRate == 230400) {
    cfsetispeed(&attr, B230400);
    cfsetospeed(&attr, B230400);
  } else if (baudRate == 921600) {
    cfsetispeed(&attr, B921600);
    cfsetospeed(&attr, B921600);
  } else {
    cfsetispeed(&attr, B9600);
    cfsetospeed(&attr, B9600);
  }

  if (tcsetattr(serial, TCSAFLUSH, &attr) < 0) {
    printf("Call to tcsetattr failed \n");
    return 0;
  }

  /* install the signal handler before making the device asynchronous */
  saio.sa_handler = signal_handler_IO;
  sigemptyset(&saio.sa_mask);
  saio.sa_flags = 0;
  saio.sa_restorer = NULL;
  sigaction(SIGIO, &saio, NULL);

  printf("\tSerial Port %s Initialized at %d bits per second\n",
         hwDevice.c_str(), baudRate);
  initialized = true;
  return 1;
}
void signal_handler_IO(int status) {}

int SerialPort::spSend(char *pkt) { return write(serial, pkt, strlen(pkt)); }

int SerialPort::spSend(char *pkt, int sz) { return write(serial, pkt, sz); }

int SerialPort::spSend(const char *pkt, int sz) {
  return write(serial, pkt, sz);
}

int SerialPort::spSend(uint8_t *pkt, uint8_t sz) {
  return write(serial, pkt, sz);
}

char *SerialPort::spReceive() {
  // ignoring return value
  int n = read(serial, &buffer, SER_BUF_SZ);
  if (n == -1)
    std::cout << "Error reading from Serial Port\n" << std::endl;
  return buffer;
}

char *SerialPort::spReceive(int nbytes) {
  // ignoring return value
  int n = read(serial, &buffer, nbytes);
  if (n == -1)
    std::cout << "Error reading from Serial Port\n" << std::endl;
  return buffer;
}

char SerialPort::spReceiveSingle() {
  // ignoring return value
  uint8_t ch;
  int n = read(serial, &ch, 1);
  if (n == -1)
    std::cout << "Error reading from Serial Port\n" << std::endl;
  return ch;
}

void SerialPort::spClose() {
  std::cout << "Closing serial port" << std::endl;
  close(serial);
  initialized = false;
}
} // namespace libserial
