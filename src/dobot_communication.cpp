#include "dobot_magician_driver/dobot_communication.h"
#include <sstream>

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

void print_hex(std::vector<u_int8_t> input){  //used to print the serial command in hex and int for debugging


    for(int k=0;k<input.size();k++){
       std::cout<< int(input[k]) << " ";
    }

    std::cout<< std::endl;
    for(int k=0;k<input.size();k++){
       std::cout<< std::hex << int(input[k]) << " ";
    }
    std::cout<< std::endl;

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
                next_char = _serial_port->ReadByte(500);
                data.push_back(next_char);
            }
            catch(SerialPort::ReadTimeout &e)
            {
//                std::cout << "TIMEDOUT "<< std::endl;
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

uint64_t DobotCommunication::getQueuedCmdIndex(std::vector<u_int8_t> data)
{
    uint64_t queuedCmdIndex;
    uint8_t *packedQueueCmdIndex = &data[0];
    std::memcpy(&queuedCmdIndex, packedQueueCmdIndex, sizeof queuedCmdIndex);
    return queuedCmdIndex;
}

uint64_t DobotCommunication::setEndEffectorSuctionCup(bool is_ctrl_enabled, bool is_sucked, bool isQueued)
{
    u_int8_t ctrl = (isQueued << 1) | 0x01;
    std::vector<u_int8_t> ctrl_cmd = {0xAA,0xAA,0x04,0x3e, ctrl, is_ctrl_enabled, is_sucked};
    std::lock_guard<std::mutex> send_command_lk(_communication_mt);
    sendCommand(ctrl_cmd);
    std::vector<u_int8_t> data;
    if(!getResponse(data)){return -2;}
    if(isQueued){
        return getQueuedCmdIndex(data);
    }else{
        return -1;
    }

}

uint64_t DobotCommunication::getEndEffectorSuctionCup(std::vector<u_int8_t> &returned_data, bool isQueued)
{
    u_int8_t ctrl = (isQueued << 1) | 0x01;
    std::vector<u_int8_t> ctrl_cmd = {0xAA,0xAA,0x04,0x3e,ctrl,0x00,0x00};
    std::lock_guard<std::mutex> send_command_lk(_communication_mt);
    sendCommand(ctrl_cmd);
    std::vector<u_int8_t> data;
    if(!getResponse(data)){return -2;}
    returned_data = data;
    if(isQueued){
        return getQueuedCmdIndex(data);

    }else{
        return -1;
    }
}



uint64_t DobotCommunication::setPTPCmd(int ptpMode, std::vector<float> &target_points, bool isQueued)
{
    u_int8_t ctrl = (isQueued << 1) | 0x01;
    std::vector<u_int8_t> ctrl_cmd = {0xAA,0xAA,0x13,0x54,ctrl,0x04};
    packFromFloat(target_points,ctrl_cmd);

    std::lock_guard<std::mutex> send_command_lk(_communication_mt);
    sendCommand(ctrl_cmd);
    std::vector<u_int8_t> data;
    if(!getResponse(data)){return -2;}
    if(isQueued){
        return getQueuedCmdIndex(data);

    }else{
        return -1;
    }
}



uint64_t DobotCommunication::setEMotor(int index,int insEnabled,float speed,bool isQueued)//index 0-stepper1 1-stepper2, insEnabled 0-0ff 1-on, speed pulses/sec (+ values clockwise, - values counterclockwise)
{

    std::vector<u_int8_t> ctrl_cmd = {0xAA,0xAA,0x08,0x87,u_int8_t(isQueued),u_int8_t(index),u_int8_t(insEnabled)};
    u_int32_t speed_32_bit = u_int32_t(speed);//convert float to 32 bit hex
    std::vector<std::uint8_t> speed_8_bit( (std::uint8_t*)&speed_32_bit, (std::uint8_t*)&(speed_32_bit) + sizeof(std::uint32_t)); //split 32bit hex to 4 8bit hex
    ctrl_cmd.insert( ctrl_cmd.end(), speed_8_bit.begin(), speed_8_bit.end() ); //place the speed into ctrl_cmd
    ctrl_cmd.push_back (checksumCalc(ctrl_cmd)); //add the checksum
    std::lock_guard<std::mutex> send_command_lk(_communication_mt);
    sendCommand(ctrl_cmd);
    std::vector<u_int8_t> data;
    if(!getResponse(data)){return -2;}
    if(isQueued){
        return getQueuedCmdIndex(data);

    }else{
        return -1;
    }

}


uint64_t DobotCommunication::getPose(std::vector<u_int8_t> &returned_data, bool isQueued)
{
    u_int8_t ctrl = (isQueued << 1) | 0x01;
    std::vector<u_int8_t> ctrl_cmd = {0xAA,0xAA,0x02,0x0A,0x00};
    std::lock_guard<std::mutex> send_command_lk(_communication_mt);
    sendCommand(ctrl_cmd);
    std::vector<u_int8_t> data;
    if(!getResponse(data)){return -2;}
    returned_data = data;
    if(isQueued){
        return getQueuedCmdIndex(data);

    }else{
        return -1;
    }
}

uint64_t DobotCommunication::setHOMECmd(bool isQueued)
{
    u_int8_t ctrl = (isQueued << 1) | 0x01;
    std::vector<u_int8_t> ctrl_cmd = {0xAA,0xAA,0x06,0x1f,ctrl,0x00,0x00, 0x00, 0x00};
    std::lock_guard<std::mutex> send_command_lk(_communication_mt);
    sendCommand(ctrl_cmd);
    std::vector<u_int8_t> data;
    if(!getResponse(data)){return -2;}
    if(isQueued){
        return getQueuedCmdIndex(data);

    }else{
        return -1;
    }
}

void DobotCommunication::sendCommand(std::vector<u_int8_t> &ctrl_cmd)
{
//    std::lock_guard<std::mutex> send_command_lk(_communication_mt);
    ctrl_cmd.push_back(checksumCalc(ctrl_cmd)); //Add the checksum to the message
    _serial_port->Write(ctrl_cmd);
//    std::cout << "DOBOT_COMMUNICATION: SEND COMMAND : " << std::endl;
}

