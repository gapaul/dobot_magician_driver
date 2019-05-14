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

DobotCommunication::~DobotCommunication()
{
    _serial_port->Close();
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

bool DobotCommunication::getResponse(std::vector<u_int8_t> &returned_data)
{
    u_int8_t next_char, checksum;
    std::vector<u_int8_t> data;
    bool timeout = false;

    {
//        std::lock_guard<std::mutex> send_command_lk(_communication_mt);
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

//    for(int i = 0; i < 5; ++i)          //remove header, lenth, ID and Control (only left with payload
//    {
//        data.erase(data.begin());
//    }

    returned_data = std::vector<u_int8_t>(data.begin()+5, data.end()/*-1 use this if no pop back*/);
    return true;
}

void DobotCommunication::packFromFloat(std::vector<float> &value_to_pack, std::vector<u_int8_t> &packed_floats)
{

    for(int i = 0; i < value_to_pack.size(); ++i){
        u_int8_t bytes_temp[4];
        floatToByte(value_to_pack[i], bytes_temp);
        for(int j = 0; j < 4; ++j){
            packed_floats.push_back(bytes_temp[j]);
//            std::cout << "byte " << j << ": " << std::hex << (int)(bytes_temp[j]) << std::endl;

        }

    }

}

void DobotCommunication::floatToByte(float float_variable, u_int8_t temp_bytes[])
{
    union {
        float a;
        u_int8_t bytes[4];
    } link;
    link.a = float_variable;
    std::memcpy(temp_bytes, link.bytes, 4);
}

bool DobotCommunication::setEndEffectorSuctionCup(bool is_ctrl_enabled, bool is_sucked, bool isQueued)
{
    u_int8_t ctrl = (isQueued << 4) | 0x01;
    std::vector<u_int8_t> ctrl_cmd = {0xAA,0xAA,0x04,0x3e,ctrl, is_ctrl_enabled, is_sucked};
    std::lock_guard<std::mutex> send_command_lk(_communication_mt);
    sendCommand(ctrl_cmd);
    std::vector<u_int8_t> data;
    if(!getResponse(data)){return false;}
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

bool DobotCommunication::getEndEffectorSuctionCup(std::vector<u_int8_t> &returned_data, bool isQueued)
{
    u_int8_t ctrl = (isQueued << 4) | 0x00;
    std::vector<u_int8_t> ctrl_cmd = {0xAA,0xAA,0x04,0x3e,ctrl,0x00,0x00};
    std::lock_guard<std::mutex> send_command_lk(_communication_mt);
    sendCommand(ctrl_cmd);
    std::vector<u_int8_t> data;
    if(!getResponse(data)){return false;}
    returned_data = data;
    return true;
}

bool DobotCommunication::setPTPCmd(int ptpMode, std::vector<float> &target_points, bool isQueued)
{
    u_int8_t ctrl = (isQueued << 4) | 0x01;
    std::vector<u_int8_t> ctrl_cmd = {0xAA,0xAA,0x13,0x54,ctrl,0x04};
    packFromFloat(target_points,ctrl_cmd);
    std::lock_guard<std::mutex> send_command_lk(_communication_mt);
    sendCommand(ctrl_cmd);
    std::vector<u_int8_t> data;
    if(!getResponse(data)){return false;}
    return true;

}

bool DobotCommunication::getPose(std::vector<u_int8_t> &returned_data)
{
    std::vector<u_int8_t> ctrl_cmd = {0xAA,0xAA,0x02,0x0A,0x00};
    std::lock_guard<std::mutex> send_command_lk(_communication_mt);
    sendCommand(ctrl_cmd);
    std::vector<u_int8_t> data;
    if(!getResponse(data)){return false;}
    returned_data = data;
    return true;

}

bool DobotCommunication::setHOMECmd(bool isQueued)
{
    u_int8_t ctrl = (isQueued << 4) | 0x01;
    std::vector<u_int8_t> ctrl_cmd = {0xAA,0xAA,0x06,0x1f,ctrl,0x00,0x00, 0x00, 0x00};
    std::lock_guard<std::mutex> send_command_lk(_communication_mt);
    sendCommand(ctrl_cmd);
    std::vector<u_int8_t> data;
    if(!getResponse(data)){return false;}
    return true;
}

void DobotCommunication::sendCommand(std::vector<u_int8_t> &ctrl_cmd)
{
//    std::lock_guard<std::mutex> send_command_lk(_communication_mt);
    ctrl_cmd.push_back(checksumCalc(ctrl_cmd)); //Add the checksum to the message
    _serial_port->Write(ctrl_cmd);
//    std::cout << "DOBOT_COMMUNICATION: SEND COMMAND : " << std::endl;
}

