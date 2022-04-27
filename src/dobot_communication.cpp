#include "dobot_magician_driver/dobot_communication.h"

DobotCommunication::DobotCommunication() : baud_(LibSerial::BaudRate::BAUD_115200),
                                           stop_bit_(LibSerial::StopBits::STOP_BITS_1),
                                           parity_(LibSerial::Parity::PARITY_NONE),
                                           character_size_(LibSerial::CharacterSize::CHAR_SIZE_8),
                                           serial_timeout_(SERIAL_TIMEOUT),
                                           try_limit_(TRY_LIMIT)
{
    serial_port_ = new LibSerial::SerialPort();
    product_id_ = 0x7523;
    vendor_id_ = 0x1a86;
}

DobotCommunication::~DobotCommunication()
{
    serial_port_->Close();
}

void DobotCommunication::init(std::string port)
{
    // Evaluate the port first
    // Wait until port is ready or until timeout
    unsigned long start_time = std::chrono::duration_cast<std::chrono::seconds>(
                            std::chrono::system_clock::now().time_since_epoch()).count();
    while (true)
    {
        unsigned long current_time = std::chrono::duration_cast<std::chrono::seconds>(
                                    std::chrono::system_clock::now().time_since_epoch()).count();
        // If the dobot usb port is detected then we resume with operation
        if(portReady())
        {
            break;
        }
        // If this has waited for 60s => exit immediately
        if(current_time - start_time > 60)
        {
            exit(0);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    port_ = port;
}

bool DobotCommunication::startConnection()
{
    serial_port_->Open(port_);
    serial_port_->SetBaudRate(baud_);
    serial_port_->SetStopBits(stop_bit_);
    serial_port_->SetParity(parity_);
    serial_port_->SetCharacterSize(character_size_);
    return serial_port_->IsOpen();
}

bool DobotCommunication::isConnected()
{
    bool serial_state = false;

    try
    {
        serial_state = serial_port_->IsOpen();
    }
    catch (const std::exception &e)
    {
        std::cout << "Failed to query serial state" << std::endl;
        serial_state = false;
    }

    return serial_state;
}

bool DobotCommunication::closeConnection()
{
    bool serial_state = false;

    try
    {
        serial_port_->Close();
        serial_state = serial_port_->IsOpen();
    }
    catch (const std::exception &e)
    {
        std::cout << "Failed to query serial state" << std::endl;
        serial_state = false;
    }

    return serial_state;
}

uint8_t DobotCommunication::checksumCalc(std::vector<uint8_t> &ctrl_cmd)
{
    uint8_t checksum = 0;

    for (int i = 0; i < ctrl_cmd.size(); ++i)
    { //Add the values of the 'payload' only
        checksum += ctrl_cmd.at(i);
    }

    checksum = 256 - checksum; //two's complement

    return checksum;
}

bool DobotCommunication::getResponse(std::vector<uint8_t> &returned_payload)
{
    uint8_t next_char, length_payload, checksum;
    std::vector<uint8_t> payload;
    bool read_success = false;
    int try_iteration = 0;
    int header_size = 0;
    unsigned char header = 0xAA;

    // find the header
    while (header_size < 2)
    {
        read_success = tryReadByte(next_char);
        if (read_success && next_char == header)
        {
            ++header_size;
        }
        else
        {
            header_size = 0;
            ++try_iteration;
        }

        if (!read_success || try_iteration > try_limit_)
        {
            return false;
        }
    }

    // get length
    if (tryReadByte(next_char))
    {
        length_payload = next_char;
    }
    else
    {
        return false;
    }

    // get payload
    for (int i = 0; i < length_payload; ++i)
    {
        read_success = tryReadByte(next_char);
        if (read_success)
        {
            payload.push_back(next_char);
        }
        else
        {
            return false;
        }
    }

    // get checksum
    if (read_success && tryReadByte(next_char))
    {
        checksum = (next_char);
    }
    else
    {
        return false;
    }

    //    printf("checksum %x - data %x\n", checksum, checksumCalc(data));
    if (checksum != checksumCalc(payload))
    {
        return false;
    }

    returned_payload = payload; // std::vector<uint8_t>(data.begin() + 5, data.end() /*-1 use this if no pop back*/);

    return true;
}

void DobotCommunication::packFromFloat(std::vector<float> &value_to_pack, std::vector<uint8_t> &packed_floats)
{

    for (int i = 0; i < value_to_pack.size(); ++i)
    {
        uint8_t bytes_temp[4];
        floatToByte(value_to_pack[i], bytes_temp);
        for (int j = 0; j < 4; ++j)
        {
            packed_floats.push_back(bytes_temp[j]);
            //            std::cout << "byte " << j << ": " << std::hex << (int)(bytes_temp[j]) << std::endl;
        }
    }
}

void DobotCommunication::packFromDouble(std::vector<double> &value_to_pack, std::vector<uint8_t> &packed_doubles)
{
    for (int i = 0; i < value_to_pack.size(); ++i)
    {
        uint8_t bytes_temp[8];
        doubleToByte(value_to_pack[i], bytes_temp);
        for (int j = 0; j < 8; j++)
        {
            packed_doubles.push_back(bytes_temp[j]);
        }
    }
}

void DobotCommunication::floatToByte(float float_variable, uint8_t temp_bytes[])
{
    union
    {
        float a;
        uint8_t bytes[4];
    } link;
    link.a = float_variable;
    std::memcpy(temp_bytes, link.bytes, 4);
}

void DobotCommunication::doubleToByte(double double_variable, uint8_t temp_bytes[])
{
    union
    {
        double a;
        uint8_t bytes[8];
    } link;

    link.a = double_variable;
    std::memcpy(temp_bytes, link.bytes, 8);
}

uint64_t DobotCommunication::getQueuedCmdIndex(std::vector<uint8_t> payload)
{
    uint64_t queuedCmdIndex;
    uint8_t *packedQueueCmdIndex = &payload[2];
    std::memcpy(&queuedCmdIndex, packedQueueCmdIndex, sizeof queuedCmdIndex);
    return queuedCmdIndex;
}

bool DobotCommunication::setLinearRailStatus(bool is_enabled, uint8_t version, bool is_queued)
{
    uint64_t queue_cmd_index;
    return setLinearRailStatus(is_enabled, version, queue_cmd_index, is_queued);
}

bool DobotCommunication::setLinearRailStatus(bool is_enabled, uint8_t version, uint64_t &queue_cmd_index, bool is_queued)
{
    uint8_t ctrl = (is_queued << 1) | 0x01;
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA, 0x04, 0x3, ctrl, (uint8_t)is_enabled, version};
    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> payload;
    if (!getResponse(payload))
    {
        return false;
    }
    if (is_queued)
    {
        queue_cmd_index = getQueuedCmdIndex(payload);
    }
    else
    {
        queue_cmd_index = std::numeric_limits<uint64_t>::quiet_NaN();
    }
    return true;
}

bool DobotCommunication::getLinearRailStatus(std::vector<uint8_t> &returned_data)
{
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA, 0x02, 0x3, 0x00};
    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> data;
    if (!getResponse(data))
    {
        return false;
    }
    returned_data = data;
    return true;
}

bool DobotCommunication::setEndEffectorSuctionCup(bool is_ctrl_enabled, bool is_sucked, bool is_queued)
{
    uint64_t queue_cmd_index;
    return setEndEffectorSuctionCup(is_ctrl_enabled, is_sucked, queue_cmd_index, is_queued);
}

bool DobotCommunication::setEndEffectorSuctionCup(bool is_ctrl_enabled, bool is_sucked, uint64_t &queue_cmd_index, bool is_queued)
{
    uint8_t ctrl = (is_queued << 1) | 0x01;
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA, 0x04, 0x3e, ctrl, (uint8_t)is_ctrl_enabled, (uint8_t)is_sucked};

    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> payload;
    if (!getResponse(payload))
    {
        return false;
    }
    if (is_queued)
    {
        queue_cmd_index = getQueuedCmdIndex(payload);
    }
    else
    {
        queue_cmd_index = std::numeric_limits<uint64_t>::quiet_NaN();
    }
    return true;
}

bool DobotCommunication::getEndEffectorSuctionCup(std::vector<uint8_t> &returned_data)
{
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA, 0x04, 0x3e, 0x00, 0x00, 0x00};
    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> data;
    if (!getResponse(data))
    {
        return false;
    }
    returned_data = data;
    return true;
}

bool DobotCommunication::setEndEffectorGripper(bool is_ctrl_enabled, bool is_gripped, bool is_queued)
{
    uint64_t queue_cmd_index;
    return setEndEffectorGripper(is_ctrl_enabled, is_gripped, queue_cmd_index, is_queued);
}

bool DobotCommunication::setEndEffectorGripper(bool is_ctrl_enabled, bool is_gripped, uint64_t &queue_cmd_index, bool is_queued)
{
    uint8_t ctrl = (is_queued << 1) | 0x01;
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA, 0x04, 0x3f, ctrl, (uint8_t)is_ctrl_enabled, (uint8_t)is_gripped};

    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> payload;
    if (!getResponse(payload))
    {
        return false;
    }
    if (is_queued)
    {
        queue_cmd_index = getQueuedCmdIndex(payload);
    }
    else
    {
        queue_cmd_index = std::numeric_limits<uint64_t>::quiet_NaN();
    }
    return true;
}

bool DobotCommunication::setPTPWithRailCmd(int ptp_mode, std::vector<float> &target_points, bool is_queued)
{
    uint64_t queue_cmd_index;
    return setPTPWithRailCmd(ptp_mode, target_points, queue_cmd_index, is_queued);
}

bool DobotCommunication::setPTPWithRailCmd(int ptp_mode, std::vector<float> &target_points, uint64_t &queue_cmd_index, bool is_queued)
{
    uint8_t ctrl = (is_queued << 1) | 0x01;
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA, 0x17, 0x56, ctrl, (uint8_t)ptp_mode};
    packFromFloat(target_points, ctrl_cmd);

    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> payload;
    if (!getResponse(payload))
    {
        return false;
    }
    if (is_queued)
    {
        queue_cmd_index = getQueuedCmdIndex(payload);
    }
    else
    {
        queue_cmd_index = std::numeric_limits<uint64_t>::quiet_NaN();
    }
    return true;
}

bool DobotCommunication::setPTPCmd(int ptp_mode, std::vector<float> &target_points, bool is_queued)
{
    uint64_t queue_cmd_index;
    return setPTPCmd(ptp_mode, target_points, queue_cmd_index, is_queued);
}

bool DobotCommunication::setPTPCmd(int ptp_mode, std::vector<float> &target_points, uint64_t &queue_cmd_index, bool is_queued)
{
    uint8_t ctrl = (is_queued << 1) | 0x01;
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA, 0x13, 0x54, ctrl, (uint8_t)ptp_mode};
    packFromFloat(target_points, ctrl_cmd);

    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> payload;
    if (!getResponse(payload))
    {
        return false;
    }
    if (is_queued)
    {
        queue_cmd_index = getQueuedCmdIndex(payload);
    }
    else
    {
        queue_cmd_index = std::numeric_limits<uint64_t>::quiet_NaN();
    }
    return true;
}

/*
 *  JOG COMMANDS
 */

bool DobotCommunication::setJOGJointParams(std::vector<float> &jog_joint_param, bool is_queued)
{
    uint64_t queue_cmd_index;
    return setJOGJointParams(jog_joint_param, queue_cmd_index, is_queued);
}

bool DobotCommunication::setJOGJointParams(std::vector<float> &jog_joint_param, uint64_t &queue_cmd_index, bool is_queued)
{
    uint8_t ctrl = (is_queued << 1) | 0x01;
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA, 0x0F, 0x46, ctrl};
    packFromFloat(jog_joint_param, ctrl_cmd);
    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> payload;
    if (!getResponse(payload))
    {
        return false;
    }
    if (is_queued)
    {
        queue_cmd_index = getQueuedCmdIndex(payload);
    }
    else
    {
        queue_cmd_index = std::numeric_limits<uint64_t>::quiet_NaN();
    }
    return true;
}

bool DobotCommunication::getJOGJointParams(std::vector<uint8_t> &returned_data)
{
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA, 0x02, 46, 0x00};
    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> data;
    if (!getResponse(data))
    {
        return false;
    }
    returned_data = data;
    return true;
}

bool DobotCommunication::setJOGCommonParams(std::vector<float> &jog_common_param, bool is_queued)
{
    uint64_t queue_cmd_index;
    return setJOGCommonParams(jog_common_param, queue_cmd_index, is_queued);
}

bool DobotCommunication::setJOGCommonParams(std::vector<float> &jog_common_param, uint64_t &queue_cmd_index, bool is_queued)
{
    uint8_t ctrl = (is_queued << 1) | 0x01;
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA, 0x0F, 0x48, ctrl};
    packFromFloat(jog_common_param, ctrl_cmd);
    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> payload;
    if (!getResponse(payload))
    {
        return false;
    }
    if (is_queued)
    {
        queue_cmd_index = getQueuedCmdIndex(payload);
    }
    else
    {
        queue_cmd_index = std::numeric_limits<uint64_t>::quiet_NaN();
    }
    return true;
}

bool DobotCommunication::getJOGCommonParams(std::vector<uint8_t> &returned_data)
{
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA, 0x02, 0x48, 0x00};
    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> data;
    if (!getResponse(data))
    {
        return false;
    }
    returned_data = data;
    return true;
}

bool DobotCommunication::setJOGCmd(uint8_t is_joint, uint8_t cmd, bool is_queued)
{
    uint64_t queue_cmd_index;
    return setJOGCmd(is_joint, cmd, queue_cmd_index, is_queued);
}

bool DobotCommunication::setJOGCmd(uint8_t is_joint, uint8_t cmd, uint64_t &queue_cmd_index, bool is_queued)
{
    uint8_t ctrl = (is_queued << 1) | 0x01;
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA, 0x13, 0x5B, ctrl, is_joint, cmd};
    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> payload;
    if (!getResponse(payload))
    {
        return false;
    }
    if (is_queued)
    {
        queue_cmd_index = getQueuedCmdIndex(payload);
    }
    else
    {
        queue_cmd_index = std::numeric_limits<uint64_t>::quiet_NaN();
    }
    return true;
}

/*
 *  I/O COMMANDS
 */
bool DobotCommunication::setIOMultiplexing(int address, int multiplex, bool is_queued)
{
    uint64_t queue_cmd_index;
    return setIOMultiplexing(address, multiplex, queue_cmd_index, is_queued);
}

bool DobotCommunication::setIOMultiplexing(int address, int multiplex, uint64_t &queue_cmd_index, bool is_queued)
{
    uint8_t ctrl = (is_queued << 1) | 0x01;
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA, 0x04, 0x82, ctrl, (uint8_t)address, (uint8_t)multiplex};

    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> payload;
    if (!getResponse(payload))
    {
        return false;
    }
    if (is_queued)
    {
        queue_cmd_index = getQueuedCmdIndex(payload);
    }
    else
    {
        queue_cmd_index = std::numeric_limits<uint64_t>::quiet_NaN();
    }
    return true;
}

bool DobotCommunication::getIOMultiplexing(int address, std::vector<uint8_t> &returned_data)
{
    uint8_t ctrl = 0x00;
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA, 0x04, 0x82, ctrl, (uint8_t)address, 0x00};

    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> data;
    if (!getResponse(data))
    {
        return false;
    }
    returned_data = data;
    return true;
}

bool DobotCommunication::setIODO(int address, bool level, bool is_queued)
{
    uint64_t queue_cmd_index;
    return setIODO(address, level, queue_cmd_index, is_queued);
}

bool DobotCommunication::setIODO(int address, bool level, uint64_t &queue_cmd_index, bool is_queued)
{
    uint8_t ctrl = (is_queued << 1) | 0x01;
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA, 0x04, 0x83, ctrl, (uint8_t)address, (uint8_t)level};

    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> payload;
    if (!getResponse(payload))
    {
        return false;
    }
    if (is_queued)
    {
        queue_cmd_index = getQueuedCmdIndex(payload);
    }
    else
    {
        queue_cmd_index = std::numeric_limits<uint64_t>::quiet_NaN();
    }
    return true;
}

bool DobotCommunication::getIODO(int address, std::vector<uint8_t> &returned_data)
{
    uint8_t ctrl = 0x00;
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA, 0x04, 0x83, ctrl, (uint8_t)address, 0x00};

    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> data;
    if (!getResponse(data))
    {
        return false;
    }
    returned_data = data;
    return true;
}

bool DobotCommunication::setIOPWM(int address, float frequency, float duty_cycle, bool is_queued)
{
    uint64_t queue_cmd_index;
    return setIOPWM(address, frequency, duty_cycle, queue_cmd_index, is_queued);
}

bool DobotCommunication::setIOPWM(int address, float frequency, float duty_cycle, uint64_t &queue_cmd_index, bool is_queued)
{
    uint8_t ctrl = (is_queued << 1) | 0x01;
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA, 0x0B, 0x84, ctrl, (uint8_t)address};

    std::vector<float> params = {frequency, duty_cycle};
    packFromFloat(params, ctrl_cmd);

    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> payload;
    if (!getResponse(payload))
    {
        return false;
    }
    if (is_queued)
    {
        queue_cmd_index = getQueuedCmdIndex(payload);
    }
    else
    {
        queue_cmd_index = std::numeric_limits<uint64_t>::quiet_NaN();
    }
    return true;
}

bool DobotCommunication::getIODI(int address, std::vector<uint8_t> &returned_data)
{
    uint8_t ctrl = 0x00;
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA, 0x04, 0x85, ctrl, (uint8_t)address, 0x00};

    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> data;
    if (!getResponse(data))
    {
        return false;
    }
    returned_data = data;
    return true;
}

bool DobotCommunication::getIOADC(int address, std::vector<uint8_t> &returned_data)
{
    uint8_t ctrl = 0x00;
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA, 0x05, 0x86, ctrl, (uint8_t)address, 0x00, 0x00};

    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> data;
    if (!getResponse(data))
    {
        return false;
    }
    returned_data = data;
    return true;
}

bool DobotCommunication::setEMotor(int index, bool is_enabled, int32_t speed, bool is_queued)
{
    uint64_t queue_cmd_index;
    return setEMotor(index, is_enabled, speed, queue_cmd_index, is_queued);
}

bool DobotCommunication::setEMotor(int index, bool is_enabled, int32_t speed, uint64_t &queue_cmd_index, bool is_queued)
{
    uint8_t ctrl = (is_queued << 1) | 0x00;
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA, 0x08, 0x87, ctrl, (uint8_t)index, (uint8_t)is_enabled};
    speed = (uint32_t)speed;
    std::vector<uint8_t> speed_8_bit((uint8_t *)&(speed), (uint8_t *)&(speed) + sizeof(uint32_t)); //split 32bit hex to 4 8bit hex
    ctrl_cmd.insert(ctrl_cmd.end(), speed_8_bit.begin(), speed_8_bit.end());                       //place the speed into ctrl_cmd
    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> payload;
    if (!getResponse(payload))
    {
        return false;
    }
    if (is_queued)
    {
        queue_cmd_index = getQueuedCmdIndex(payload);
    }
    else
    {
        queue_cmd_index = std::numeric_limits<uint64_t>::quiet_NaN();
    }
    return true;
}

bool DobotCommunication::setCPParams(std::vector<float> &cp_params, bool real_time_track, bool is_queued)
{
    uint64_t queue_cmd_index;
    return setCPParams(cp_params, real_time_track, queue_cmd_index, is_queued);
}

bool DobotCommunication::setCPParams(std::vector<float> &cp_params, bool real_time_track, uint64_t &queue_cmd_index, bool is_queued)
{
    uint8_t ctrl = (is_queued << 1) | 0x01;
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA, 0x0F, 0x5A, ctrl};
    packFromFloat(cp_params, ctrl_cmd);
    ctrl_cmd.push_back((uint8_t)real_time_track);
    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> payload;
    if (!getResponse(payload))
    {
        return false;
    }
    if (is_queued)
    {
        queue_cmd_index = getQueuedCmdIndex(payload);
    }
    else
    {
        queue_cmd_index = std::numeric_limits<uint64_t>::quiet_NaN();
    }
    return true;
}

bool DobotCommunication::getCPParams(std::vector<uint8_t> &returned_data)
{
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA, 0x02, 0x5A, 0x00};
    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> data;
    if (!getResponse(data))
    {
        return false;
    }
    returned_data = data;
    return true;
}

bool DobotCommunication::setCPCmd(std::vector<float> &cp_cmd, bool cp_mode, bool is_queued)
{
    uint64_t queue_cmd_index;
    return setCPCmd(cp_cmd, cp_mode, queue_cmd_index, is_queued);
}

bool DobotCommunication::setCPCmd(std::vector<float> &cp_cmd, bool cp_mode, uint64_t &queue_cmd_index, bool is_queued)
{
    uint8_t ctrl = (is_queued << 1) | 0x01;
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA, 0x13, 0x5B, ctrl, (uint8_t)cp_mode};
    packFromFloat(cp_cmd, ctrl_cmd);
    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> payload;
    if (!getResponse(payload))
    {
        return false;
    }
    if (is_queued)
    {
        queue_cmd_index = getQueuedCmdIndex(payload);
    }
    else
    {
        queue_cmd_index = std::numeric_limits<uint64_t>::quiet_NaN();
    }
    return true;
}

bool DobotCommunication::getPose(std::vector<uint8_t> &returned_data)
{
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA, 0x02, 0x0A, 0x00};
    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> payload;
    if (!getResponse(payload))
    {
        return false;
    }
    returned_data = payload;
    return true;
}

bool DobotCommunication::getRailPose(std::vector<uint8_t> &returned_data)
{
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA, 0x02, 0x0D, 0x00};
    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> payload;
    if (!getResponse(payload))
    {
        return false;
    }
    returned_data = payload;
    return true;
}

bool DobotCommunication::setHOMECmd(bool is_queued)
{
    uint64_t queue_cmd_index;
    return setHOMECmd(queue_cmd_index, is_queued);
}

bool DobotCommunication::setHOMECmd(uint64_t &queue_cmd_index, bool is_queued)
{
    uint8_t ctrl = (is_queued << 1) | 0x01;
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA, 0x06, 0x1f, ctrl, 0x00, 0x00, 0x00, 0x00};
    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> payload;
    if (!getResponse(payload))
    {
        return false;
    }
    if (is_queued)
    {
        queue_cmd_index = getQueuedCmdIndex(payload);
    }
    else
    {
        queue_cmd_index = std::numeric_limits<uint64_t>::quiet_NaN();
    }
    return true;
}

bool DobotCommunication::setQueuedCmdStartExec(void)
{
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA, 0x02, 0xF0, 0x01};
    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> payload;
    if (!getResponse(payload))
    {
        return false;
    }
    return true;
}

bool DobotCommunication::setQueuedCmdStopExec(void)
{
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA, 0x02, 0xF1, 0x01};
    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> payload;
    if (!getResponse(payload))
    {
        return false;
    }
    return true;
}

bool DobotCommunication::setQueuedCmdForceStopExec(void)
{
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA, 0x02, 0xF2, 0x01};
    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> payload;
    if (!getResponse(payload))
    {
        return false;
    }
    return true;
}

bool DobotCommunication::setQueuedCmdClear(void)
{
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA, 0x02, 0xF5, 0x01};
    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> payload;
    if (!getResponse(payload))
    {
        return false;
    }
    return true;
}

void DobotCommunication::sendCommand(std::vector<uint8_t> &ctrl_cmd)
{
    std::vector<uint8_t> payload(ctrl_cmd.begin() + 3, ctrl_cmd.end());
    ctrl_cmd.push_back(checksumCalc(payload)); //Add the checksum to the message
    try
    {
        serial_port_->Write(ctrl_cmd);
    }
    catch (LibSerial::NotOpen &e)
    {
    }
    catch (std::runtime_error &e)
    {
    }
}

bool DobotCommunication::tryReadByte(uint8_t &next_char)
{
    try
    {
        // According to the doc, ReadByte should throw exceptions if the serial port is not opened
        serial_port_->ReadByte(next_char, serial_timeout_);
        return true;
    }
    catch (LibSerial::ReadTimeout &e)
    {
        return false;
    }
    catch (LibSerial::NotOpen &e)
    {
        return false;
    }
    catch (std::runtime_error &e)
    {
        return false;
    }
}

bool DobotCommunication::sendCustomCommand(std::vector<float> cmd, uint64_t &queue_cmd_index, bool is_queued)
{
    std::vector<uint8_t> ctrl_cmd = {0xAA, 0xAA};
    packFromFloat(cmd, ctrl_cmd);
    std::lock_guard<std::mutex> send_command_lk(communication_mt_);
    sendCommand(ctrl_cmd);
    std::vector<uint8_t> payload;
    if (!getResponse(payload))
    {
        return false;
    }
    if (is_queued)
    {
        queue_cmd_index = getQueuedCmdIndex(payload);
    }
    else
    {
        queue_cmd_index = std::numeric_limits<uint64_t>::quiet_NaN();
    }
    return true;
}

bool DobotCommunication::portReady()
{
    bool ready = false;
    // Pointer to pointer to store list of usb devices
    libusb_device **usb_devices;
    libusb_context *ctx = NULL; //a libusb session
    libusb_init(&ctx);          //initialize a library session

    ssize_t cnt = libusb_get_device_list(ctx, &usb_devices); //get the list of devices
    if (cnt < 0)
    {
        std::cout << "Get Device Error" << std::endl; //there was an error
    }
    for (ssize_t i = 0; i < cnt; i++)
    {
        libusb_device_descriptor desc;
        int r = libusb_get_device_descriptor(usb_devices[i], &desc);
        if (r < 0)
        {
            // We should raise error here
        }
        if(desc.idVendor == vendor_id_ && desc.idProduct == product_id_)
        {
            std::cout << "VendorID: " << desc.idVendor << "  ";
            std::cout << "ProductID: " << desc.idProduct << std::endl;
            // Found the device. Exit the loop
            ready = true;
            break;
        }
        
    }
    libusb_free_device_list(usb_devices, 1); //free the list, unref the devices in it
    libusb_exit(ctx);                        //close the session
    return ready;
}