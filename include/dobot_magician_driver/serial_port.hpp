// ACKNOWLEDGEMENT
// Special thanks to Geoffrey Hunter and his wonderful explantions and
// code examples that helped me a lot in understanding how serial 
// communication works in ubuntu, which then allows me to build this
// simple library for using in my own project
// Links: https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/#software-flow-control-ixoff-ixon-ixany
// https://github.com/gbmhunter/CppLinuxSerial

#ifndef HANS_CUTE_DRIVER__SERIAL_PORT_HPP_
#define HANS_CUTE_DRIVER__SERIAL_PORT_HPP_

#include <string>
#include <fstream>  // For file I/O (reading/writing to COM port)
#include <sstream>
#include <chrono>
#include <thread>

#include <iostream>
#include <sstream>
#include <stdio.h>   // Standard input/output definitions
#include <string.h>  // String function definitions
#include <unistd.h>  // UNIX standard function definitions
#include <fcntl.h>   // File control definitions
#include <errno.h>   // Error number definitions

#include <system_error>  // For throwing std::system_error
#include <sys/ioctl.h>   // Used for TCGETS2, which is required for custom baud rates
#include <sys/file.h>
#include <cassert>

#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <algorithm>
#include <iterator>

#include <vector>
#include <asm/ioctls.h>
#include <asm/termbits.h>

enum class NumDataBits
{
  FIVE,
  SIX,
  SEVEN,
  EIGHT,
};

enum class Parity
{
  NONE,
  EVEN,
  ODD,
};

enum class NumStopBits
{
  ONE,
  TWO,
};

enum class SerialError
{
  WRITE_ERROR,
  READ_ERROR,
};

class SerialPort
{
public:
  SerialPort(const std::string& port, const speed_t& baud_rate, int32_t timeout);
  virtual ~SerialPort();

  // Setters
  void setPort(const std::string& port);
  void setBaudRate(const speed_t& baud_rate);
  void setNumDataBits(const NumDataBits& num_data_bits);
  void setParity(const Parity& parity);
  void setNumStopBits(const NumStopBits& num_stop_bits);
  void setTimeout(const int32_t& timeout);

  //
  bool openPort();
  bool closePort();
  bool isOpen() const;

  void write(const std::vector<uint8_t>& data);
  void wait();
  unsigned int read(std::vector<uint8_t>& data);

  int available();

private:
  void configure();
  
  std::string port_;
  speed_t baud_rate_;

  NumDataBits num_data_bits_;
  Parity parity_;
  NumStopBits num_stop_bits_;

  int file_desc_;

  int32_t timeout_;

  bool is_open_;
  bool echo_;

  std::vector<char> read_buffer_;
  unsigned char read_buffer_size_B_;
};

#endif
