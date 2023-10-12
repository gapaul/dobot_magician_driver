#include "hans_cute_driver/serial_port.hpp"

SerialPort::SerialPort(const std::string &port, const speed_t &baud_rate, int32_t timeout)
    : port_(port), baud_rate_(baud_rate), timeout_(timeout), num_data_bits_(NumDataBits::EIGHT), parity_(Parity::NONE), num_stop_bits_(NumStopBits::ONE), is_open_(false)
{
  read_buffer_size_B_ = 255;
  read_buffer_.reserve(read_buffer_size_B_);
}

SerialPort::~SerialPort()
{
  try
  {
    closePort();
  }
  catch (const std::exception &e)
  {
  }
}

void SerialPort::setPort(const std::string &port)
{
}

void SerialPort::setBaudRate(const speed_t &baud_rate)
{
}

void SerialPort::setNumDataBits(const NumDataBits &num_data_bits)
{
}

void SerialPort::setParity(const Parity &parity)
{
}

void SerialPort::setNumStopBits(const NumStopBits &num_stop_bits)
{
}

void SerialPort::setTimeout(const int32_t &timeout)
{
}

bool SerialPort::openPort()
{
  if (port_.empty())
  {
    // Thow empty error here
    return false;
  }

  file_desc_ = open(port_.c_str(), O_RDWR);

  if (file_desc_ == -1)
  {
    // Throw error here
    return false;
  }
  configure();
  is_open_ = true;
  return true;
}

bool SerialPort::closePort()
{
  if (file_desc_ != -1)
  {
    int retVal = close(file_desc_);
    if (retVal != 0)
      return false;

    file_desc_ = -1;
  }
  return true;
}

bool SerialPort::isOpen() const
{
  return is_open_;
}

void SerialPort::write(const std::vector<uint8_t> &data)
{
  if (flock(file_desc_, LOCK_EX | LOCK_NB) == -1)
  {
    throw std::runtime_error("Serial port with file descriptor " + std::to_string(file_desc_) +
                             " is already locked by another process.");
  }
  int writeResult = ::write(file_desc_, data.data(), data.size());
}

void SerialPort::wait()
{
  unsigned long start_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
          .count();
  while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
                     .count() -
                 start_time <
             timeout_ &&
         available() == 0)
    ;
}

unsigned int SerialPort::read(std::vector<uint8_t> &data)
{
  data.clear();
  if (flock(file_desc_, LOCK_EX | LOCK_NB) == -1)
  {
    throw std::runtime_error("Serial port with file descriptor " + std::to_string(file_desc_) +
                             " is already locked by another process.");
  }
  if (file_desc_ == 0)
  {
    return 0;
  }
  ssize_t n = ::read(file_desc_, &read_buffer_[0], read_buffer_size_B_);
  if (n > 0)
  {
    std::copy(read_buffer_.begin(), read_buffer_.begin() + n, std::back_inserter(data));
  }
  return n;
}

int SerialPort::available()
{
  int bytes;
  ioctl(file_desc_, FIONREAD, &bytes);
  return bytes;
}

// Private
void SerialPort::configure()
{
  struct termios2 tty;
  ioctl(file_desc_, TCGETS2, &tty);

  //================= (Control Mode) =================//
  // Parity
  switch (parity_)
  {
  case Parity::NONE:
    tty.c_cflag &= ~PARENB;
    break;
  case Parity::EVEN:
    tty.c_cflag |= PARENB;
    tty.c_cflag &= ~PARODD; // Clearing PARODD makes the parity even
    break;
  case Parity::ODD:
    tty.c_cflag |= PARENB;
    tty.c_cflag |= PARODD;
    break;
  default:
    // Default to disable parity
    tty.c_cflag &= ~PARENB;
  }

  // Num stop bits
  switch (num_stop_bits_)
  {
  case NumStopBits::ONE:
    tty.c_cflag &= ~CSTOPB;
    break;
  case NumStopBits::TWO:
    tty.c_cflag |= CSTOPB;
    break;
  default:
    // Default to one stop bit
    tty.c_cflag &= ~CSTOPB;
  }

  // Num of data bits
  tty.c_cflag &= ~CSIZE; // CSIZE is a mask for the number of bits per character
  switch (num_data_bits_)
  {
  case NumDataBits::FIVE:
    tty.c_cflag |= CS5;
    break;
  case NumDataBits::SIX:
    tty.c_cflag |= CS6;
    break;
  case NumDataBits::SEVEN:
    tty.c_cflag |= CS7;
    break;
  case NumDataBits::EIGHT:
    tty.c_cflag |= CS8;
    break;
  default:
    // Default to 8 data bits
    tty.c_cflag |= CS8;
  }

  tty.c_cflag &= ~CRTSCTS;       // Disable hadrware flow control (RTS/CTS)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  //================= (Local Mode) =================//
  // Set to non-canonical mode
  tty.c_lflag &= ~ICANON;

  // Disable echo
  tty.c_lflag &= ~ECHO;   // Disable echo
  tty.c_lflag &= ~ECHOE;  // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo

  // Disable signal characters
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

  //================= (Input Mode) =================//
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  //================= (Output Mode) =================//
  tty.c_oflag = 0;
  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  // tty.c_oflag &= ~ONLCR;  // Prevent conversion of newline to carriage return/line feed

  //================= (VMIN VTIME) =================//
  // Timeout will be manually handled by the class
  // Setting both to 0 will give a non-blocking read
  tty.c_cc[VTIME] = 0;
  tty.c_cc[VMIN] = 1;

  //================= (Baud rate) =================//
  tty.c_cflag &= ~CBAUD;
  tty.c_cflag |= CBAUDEX;

  tty.c_ispeed = baud_rate_;
  tty.c_ospeed = baud_rate_;

  ioctl(file_desc_, TCSETS2, &tty);
}