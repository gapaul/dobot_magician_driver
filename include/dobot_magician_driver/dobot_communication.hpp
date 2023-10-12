#ifndef DOBOT_MAGICIAN_DRIVER__DOBOT_COMMUNICATION_HPP_
#define DOBOT_MAGICIAN_DRIVER__DOBOT_COMMUNICATION_HPP_

#include <iostream>
#include <mutex>
#include <string>
#include <vector>
#include <cstring>
#include <limits>
#include <cmath>

#include "serial_port.hpp"
#include "dobot_utils.h"
#include "dobot_const.hpp"

namespace DobotMagician
{
  class DobotCommunication
  {
  public:
    DobotCommunication();
    ~DobotCommunication();

    bool open(const std::string &port);
    bool close();
    bool ping();

    bool setEndEffectorSuctionCup(const bool &pump_enable, const bool &control_enable);
    bool getEndEffectorSuctionCup(std::vector<uint8_t> &returned_data);

    bool setEndEffectorGripper(const bool &pump_enable, const bool &grip);
    bool getEndEffectorSuctionCup(std::vector<uint8_t> &returned_data);

    bool setPTPCmd(PTPMode ptp_mode, std::vector<float> &target_points);
    bool setPTPWithRailCmd(PTPMode ptp_mode, std::vector<float> &target_points);

  protected:
    bool read(const uint8_t &address, const uint8_t &size, const bool &is_queued,
              std::vector<uint8_t> &returned_data, unsigned long &timestamp);
    bool write(const uint8_t &address, const std::vector<uint8_t> &data,
               const bool &is_queued, const uint8_t &size,
               std::vector<uint8_t> &returned_data, unsigned long &timestamp);
    uint8_t calcCheckSum(std::vector<uint8_t> &data) const;
    int readResponse(std::vector<uint8_t> &response);
    int writeCommand(const std::vector<uint8_t> &command);

    std::shared_ptr<SerialPort> serial_port_ptr_;
  };

  enum class SerialError
  {
    NO_ERROR,

    OPEN_ERROR,
    CLOSE_ERROR,

    READ_ERROR,
    WRITE_ERROR,

    NO_RESPONSE,
    WRONG_HEADER,
    WRONG_CHECKSUM,
  };

  struct JointParams
  {
    double min;
    double max
  };
};