#include "dobot_magician_driver/dobot_communication.h"

DobotCommunication::DobotCommunication(std::string port) :
    _baud(SerialPort::BAUD_115200),
    _stop_bit(SerialPort::STOP_BITS_1),
    _parity(SerialPort::PARITY_NONE),
    _character_size(SerialPort::CHAR_SIZE_8)
{
    _serial_port = new SerialPort(port);
    _serial_port->Open();
    _serial_port->SetBaudRate(_baud);
    _serial_port->SetNumOfStopBits(_stop_bit);
    _serial_port->SetParity(_parity);
    _serial_port->SetCharSize(_character_size);

    std::cout << "Port " << port << " is " << (_serial_port->IsOpen() ? "Connected" : "Disconnected") << std::endl;
}

u_int8_t DobotCommunication::checksumCalc(std::vector<uint8_t> &ctrl_cmd)
{
    u_int8_t checksum = 0;

    for(int i = 3; i < ctrl_cmd.size(); ++i){   //Add the values of the 'payload' only
        checksum += ctrl_cmd.at(i);
    }

    checksum = 256 - checksum;                  //two's complement
    while(checksum < 0){checksum -= 256;}

//    printf("DOBOT COMMNICATION: CHECKSUM : %x \n",checksum);

    return checksum;
}

bool DobotCommunication::getResponse(std::vector<u_int8_t> &returnedData)
{
    u_int8_t next_char, checksum;
    std::vector<u_int8_t> data;
    bool timeout = false;

    while(!timeout){
        try
        {
            next_char = _serial_port->ReadByte(25);
            data.push_back(next_char);
        }
        catch(SerialPort::ReadTimeout &e)
        {
            timeout = true;
        }
    }
    if(data.size() == 0)
    {
        return false;
    }
    checksum = data.back();
    data.pop_back();
//    printf("checksum %x - data %x\n", checksum, checksumCalc(data));
    if(checksum != checksumCalc(data))
    {
        return false;
    }

    for(int i = 0; i < 5; ++i)          //remove header, lenth, ID and Control (only left with payload
    {
        data.erase(data.begin());
    }

    returnedData = data;
    return true;
}

bool DobotCommunication::setEndEffectorSuctionCup(bool is_ctrl_enabled, bool is_sucked)
{
    std::vector<u_int8_t> ctrl_cmd = {0xAA,0xAA,0x04,0x3e,0x11, is_ctrl_enabled, is_sucked};
    sendCommand(ctrl_cmd);
    return true;
}

bool DobotCommunication::setEndEffectorSuctionCup(bool is_ctrl_enabled, bool is_sucked, std::vector<u_int8_t> &returned_data)
{
    setEndEffectorSuctionCup(is_ctrl_enabled, is_sucked);

    std::vector<u_int8_t> data;
    if(!getResponse(data)){return false;}
    returned_data = data;
    return true;
}

bool DobotCommunication::getPose(std::vector<u_int8_t> &returned_data)
{
    std::vector<u_int8_t> ctrl_cmd = {0xAA,0xAA,0x02,0x0A,0x00};
    sendCommand(ctrl_cmd);
    std::vector<u_int8_t> data;
    if(!getResponse(data)){return false;}
    returned_data = data;
    return true;

}

void DobotCommunication::sendCommand(std::vector<u_int8_t> &ctrl_cmd)
{
    ctrl_cmd.push_back(checksumCalc(ctrl_cmd)); //Add the checksum to the message
    _serial_port->Write(ctrl_cmd);
//    std::cout << "DOBOT_COMMUNICATION: SEND COMMAND : " << std::endl;
}

