#include "dobot_magician_driver/dobot_communication.hpp"

namespace DobotMagician
{
  DobotCommunication::DobotCommunication()
  {
  }

  DobotCommunication::~DobotCommunication()
  {
  }
  bool DobotCommunication::open(const std::string &port)
  {
  }
  bool DobotCommunication::close()
  {
  }
  bool DobotCommunication::ping()
  {
  }

  bool DobotCommunication::setEndEffectorSuctionCup(const bool &pump_enable, const bool &control_enable)
  {
  }
  bool DobotCommunication::getEndEffectorSuctionCup(std::vector<uint8_t> &returned_data)
  {
  }

  bool DobotCommunication::setEndEffectorGripper(const bool &pump_enable, const bool &grip)
  {
  }
  bool DobotCommunication::getEndEffectorSuctionCup(std::vector<uint8_t> &returned_data)
  {
  }

  bool DobotCommunication::setPTPCmd(PTPMode ptp_mode, std::vector<float> &target_points)
  {
  }

  bool DobotCommunication::setPTPWithRailCmd(PTPMode ptp_mode, std::vector<float> &target_points)
  {
  }

  bool DobotCommunication::read(
      const uint8_t &address, const uint8_t &size, const bool &is_queued,
      std::vector<uint8_t> &returned_data, unsigned long &timestamp)
  {
    uint8_t ctrl = (is_queued << 1) | (uint8_t)INSTRUCTION::READ;
    std::vector<uint8_t> packet = {0xAA, 0xAA, 0x02, address, ctrl};
    // Calculate checksum
    std::vector<uint8_t> payload(packet.begin() + 3, packet.end());
    packet.push_back(calcCheckSum(payload));

    // Read response and return data
    int status = readResponse(returned_data);
    if (status != 0)
    {
      return false;
    }
    return true;
  }

  bool DobotCommunication::write(
      const uint8_t &address, const std::vector<uint8_t> &data, const bool &is_queued,
      const uint8_t &size, std::vector<uint8_t> &returned_data, unsigned long &timestamp)
  {
    uint8_t ctrl = (is_queued << 1) | (uint8_t)INSTRUCTION::WRITE;
    uint8_t length = data.size() + 2; // Add headers
    std::vector<uint8_t> packet = {0xAA, 0xAA, length, address, ctrl};

    // Insert data
    packet.insert(std::end(packet), std::begin(data), std::end(data));

    // Calculate checksum
    std::vector<uint8_t> payload(packet.begin() + 3, packet.end());
    packet.push_back(calcCheckSum(payload));

    // Write data
    int write_status = writeCommand(packet);
    if (write_status != (int)SerialError::NO_ERROR)
    {
      return false;
    }

    // Read response and return data
    int status = readResponse(returned_data);
    if (status != 0)
    {
      return false;
    }
    return true;
  }

  int DobotCommunication::readResponse(std::vector<uint8_t> &response)
  {
    std::vector<uint8_t> returned_data;
    if (serial_port_ptr_->available() == 0)
    {
      return (int)SerialError::NO_RESPONSE;
    }

    // To support testing this should be a separate function
    unsigned int num_byte_read = 0;
    for (int i = 0; i < 5; i++)
    {
      num_byte_read = serial_port_ptr_->read(returned_data);
      // If we actually receive data
      if (num_byte_read > 0)
      {
        break;
      }
    }
    // Unable to get a response
    if (num_byte_read == 0)
    {
      return (int)SerialError::READ_ERROR;
    }
    // Verify header first
    std::vector<uint8_t> headers = {0xAA, 0xAA};
    for (int i = 0; i < headers.size(); i++)
    {
      if (returned_data.at(i) != headers.at(i))
      {
        return (int)SerialError::WRONG_HEADER;
      }
    }

    // Then verify checksum
    if (calcCheckSum(returned_data) != returned_data.at(returned_data.size() - 1))
    {
      return (int)SerialError::WRONG_CHECKSUM;
    }
    response = returned_data;
    return (int)SerialError::NO_ERROR;
  }

  int DobotCommunication::writeCommand(const std::vector<uint8_t> &command)
  {
    try
    {
      serial_port_ptr_->write(command);
      serial_port_ptr_->wait();
      return (int)SerialError::NO_ERROR;
    }
    catch (const std::exception &se)
    {
      // Handle errors
      return (int)SerialError::WRITE_ERROR;
    }
  }

  uint8_t DobotCommunication::calcCheckSum(std::vector<uint8_t> &data) const
  {
    uint8_t checksum = 0;
    for (int i = 0; i < data.size(); ++i)
    { // Add the values of the 'payload' only
      checksum += data.at(i);
    }
    checksum = 256 - checksum; // two's complement
    return checksum;
  }
}