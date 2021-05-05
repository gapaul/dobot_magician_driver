#include <dobot_magician_driver/dobot_state.h>

DobotStates::DobotStates()
{
    current_joint_config_buffer_.received = false;
    current_end_effector_pose_buffer_.received = false;

    pause_update_thread_ = false;

    cp_params_.received = false;
    cp_params_.user_set = false;

    start_joint_angle_ = {0,0.4,0.3,0};

    for(int address = 0; address <= 20; address ++)
    {
        io_state_.io_mux.push_back(IODummy);
        io_state_.data.push_back(0);
    }

    // Initialise flag
    is_homed_ = false;
    is_on_rail_ = false;

    is_connected_ = false;
    is_e_stopped_ = false;
}

DobotStates::~DobotStates()
{
    update_state_thread_->join();
    delete update_state_thread_;
}

void DobotStates::init(std::shared_ptr<DobotCommunication> dobot_serial_ptr)
{
    dobot_serial_ = dobot_serial_ptr;
}

void DobotStates::run()
{
    // Kick the thread
    update_state_thread_ = new std::thread(&DobotStates::updateRobotStatesThread, this);
}

bool DobotStates::isConnected()
{
    return is_connected_;
}

JointConfiguration DobotStates::getRobotCurrentJointConfiguration()
{
    JointConfiguration current_joint_config;
    
    current_joint_config_buffer_.mtx.lock();
    current_joint_config = current_joint_config_buffer_.joint_data.back();
    current_joint_config_buffer_.mtx.unlock();

    return current_joint_config;
}

Pose DobotStates::getRobotCurrentEndEffectorPose()
{
    Pose current_ee_pose;

    current_end_effector_pose_buffer_.mtx.lock();
    current_ee_pose = current_end_effector_pose_buffer_.pose_data.back();
    current_end_effector_pose_buffer_.mtx.unlock();

    return current_ee_pose;
}

Pose DobotStates::getCurrentRailPosition()
{
    Pose rail_position;
    rail_position.x = 0;
    rail_position.y = 0;
    rail_position.z = 0;
    rail_position.theta = 0;

    if(!current_rail_position_buffer_.received)
    {
        return rail_position;
    }

    current_rail_position_buffer_.mtx.lock();
    rail_position = current_rail_position_buffer_.pose_data.back();
    current_rail_position_buffer_.mtx.unlock();
    return rail_position;
}

void DobotStates::setContinuosPathParams(ContinuousPathParams cp_params)
{
    cp_params_.mtx.lock();

    cp_params_.plan_accel = cp_params.plan_accel;
    cp_params_.junction_velocity = cp_params.junction_velocity;
    cp_params_.actual_accel = cp_params.actual_accel;
    cp_params_.real_time_track = cp_params.real_time_track;
    
    cp_params_.received = true;
    cp_params_.user_set = true;
    
    cp_params_.mtx.unlock();
}

bool DobotStates::unpackPose(std::vector<uint8_t> &data, std::vector<double> &pose)
{
    if(data.size() != 34){
        return false;
    }

    pose.clear();
    for(int i = 0; i < 8; ++i){
        pose.push_back(unpackFloat(data.begin()+2 + i*4));
    }
    return true;
}

float DobotStates::unpackFloat(std::vector<uint8_t>::iterator it)
{
    float temp;
    uint8_t b[] = {*it, *(it+1), *(it+2), *(it+3)};
    std::memcpy(&temp, &b, sizeof(temp)); //convert to float from bytes[4]
//    printf("%f\n", temp);
    return temp;
}

bool DobotStates::unpackCPParams(std::vector<uint8_t> &data, std::vector<float> &cp_params, uint8_t &real_time_track)
{
    if (data.size() != 15)
    {
        return false;
    }
    cp_params.clear();
    
    for (int i = 0; i < 3; ++i)
    {
        cp_params.push_back(unpackFloat(data.begin()+2 + i*4));
    }

    real_time_track = (uint8_t)*(data.begin()+14);
    return true;
}

void DobotStates::updateRobotStatesThread()
{
    std::vector<uint8_t> raw_serial_data;
    std::vector<double> config_data;
    
    Pose pose_data;
    Pose pre_pose_data;

    Pose rail_pos_data;
    rail_pos_data.x = 0;
    rail_pos_data.z = 0;
    rail_pos_data.theta = 0;

    JointConfiguration joint_data;
    JointConfiguration pre_joint_data;

    joint_data.position = std::vector<double>({0,0,0,0});
    joint_data.velocity = std::vector<double>({0,0,0,0});

    auto start_time = std::chrono::system_clock::now();
    auto end_time = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds;

    std::vector<uint8_t> io_data;
    
    while(true)
    {
        if(!dobot_serial_->isConnected())
        {
            is_connected_ = false;
            std::cout << "Unable to get a connection" << std::endl;
            safety_state_.mtx.lock();
            safety_state_.safety_state = DISCONNECTED;
            safety_state_.mtx.unlock();
            continue;
        }
        else
        {
            is_connected_ = true;
        }
        

        bool result = dobot_serial_->getPose(raw_serial_data);

        if(result)
        {
            // std::cout << unpackPose(raw_serial_data,config_data) << std::endl;
            if(unpackPose(raw_serial_data,config_data))
            {
                // #### UPDATE END EFFECTOR STATE ####
                current_end_effector_pose_buffer_.mtx.lock();
                
                pose_data.x = config_data.at(0);
                pose_data.y = config_data.at(1);
                pose_data.z = config_data.at(2);
                pose_data.theta = config_data.at(3);

                current_end_effector_pose_buffer_.pose_data.push_back(pose_data);

                if(current_end_effector_pose_buffer_.pose_data.size() > MAX_BUFFER_SIZE)
                {
                    current_end_effector_pose_buffer_.pose_data.pop_front();
                }
                
                // TODO: velocity calculation
                if(current_end_effector_pose_buffer_.pose_data.size() > 1)
                {

                }

                current_end_effector_pose_buffer_.received = true;
                current_end_effector_pose_buffer_.mtx.unlock();


                // #### UPDATE JOINT STATE ####
                current_joint_config_buffer_.mtx.lock();
                
                joint_data.position = std::vector<double>(config_data.begin() + 4,config_data.end());

                if(current_joint_config_buffer_.joint_data.size() > 1)
                {
                    pre_joint_data = current_joint_config_buffer_.joint_data.back();
                    
                    for(int i = 0; i < pre_joint_data.position.size(); i ++)
                    {

                        end_time = std::chrono::system_clock::now();

                        elapsed_seconds = (end_time - start_time);

                        double duration = elapsed_seconds.count();
                        joint_data.velocity.at(i) = (joint_data.position.at(i) - pre_joint_data.position.at(i))/duration;
                    }
                }

                start_time = std::chrono::system_clock::now();
                
                current_joint_config_buffer_.joint_data.push_back(joint_data);

                if(current_joint_config_buffer_.joint_data.size() > MAX_BUFFER_SIZE)
                {
                    current_joint_config_buffer_.joint_data.pop_front();
                }

                current_joint_config_buffer_.received = true;
                // std::cout<<current_joint_config_buffer_.joint_data.back().position.at(0)<<std::endl;
                current_joint_config_buffer_.mtx.unlock();
            }
            else
            {
                std::cout << "Failed to extract pose data" << std::endl; 
                is_connected_ = false;
                // Wait for other threads to halt
                std::this_thread::sleep_for(std::chrono::milliseconds(END_OF_HOME_DELAY));
                std::cout << "Closing serial connection" << std::endl; 
                dobot_serial_->closeConnection();
                std::this_thread::sleep_for(std::chrono::milliseconds(SERIAL_RESET_DELAY));
                std::cout << "Starting serial connection" << std::endl; 
                dobot_serial_->startConnection();
                std::this_thread::sleep_for(std::chrono::milliseconds(SERIAL_RESET_DELAY));  
                while(!dobot_serial_->isConnected())
                {
                    std::cout <<  dobot_serial_->isConnected() << std::endl;
                }
                is_connected_ = true;
                std::cout << "Robot is now ready to receive commands." << std::endl; 
                continue;
            }
        }
        else
        {
            std::cout << "Failed to read pose data" << std::endl;
            is_connected_ = false;
            // Wait for other threads to halt
            std::this_thread::sleep_for(std::chrono::milliseconds(END_OF_HOME_DELAY));
            std::cout << "Closing serial connection" << std::endl; 
            dobot_serial_->closeConnection();
            std::this_thread::sleep_for(std::chrono::milliseconds(SERIAL_RESET_DELAY));
            std::cout << "Starting serial connection" << std::endl; 
            dobot_serial_->startConnection();
            std::this_thread::sleep_for(std::chrono::milliseconds(SERIAL_RESET_DELAY));  
            while(!dobot_serial_->isConnected())
            {
                std::cout <<  dobot_serial_->isConnected() << std::endl;
            }
            is_connected_ = true;
            std::cout << "Robot is now ready to receive commands." << std::endl; 
            continue;
        }

        // #### UPDATE ROBOT IO STATES #####
        io_state_.mtx.lock();
        for(int address = 1; address <= 20; address++)
        {
            result = dobot_serial_->getIOMultiplexing(address, io_data);
            
            if(result && io_data.size() > 3)
            {
                io_state_.io_mux.at(address) = (int)io_data.at(3); 
            }
            else
            {
                io_state_.io_mux.at(address) = IODummy;
            }

            switch(io_state_.io_mux.at(address))
            {
                case IODummy:
                    io_state_.data.at(address) = 0;
                    break;

                case IODO:
                    result = dobot_serial_->getIODO(address,io_data);
                    if(result && io_data.size() > 3)
                    {
                        io_state_.data.at(address) = (float)io_data.at(3);
                    }
                    else
                    {
                        io_state_.data.at(address) = 0;   
                    }
                    break;

                case IOPWM:
                    // TODO
                    // dobot_serial_->getIOPW
                    io_state_.data.at(address) = 0;
                    break;

                case IODI:
                    result = dobot_serial_->getIODI(address, io_data);
                    if(result && io_data.size() > 3)
                    {
                        io_state_.data.at(address) = (float)io_data.at(3);
                    }
                    else
                    {
                        io_state_.data.at(address) = 0;   
                    }
                    break;

                case IOADC:
                    result = dobot_serial_->getIOADC(address, io_data);
                    if(result && io_data.size() > 4)
                    {
                        io_state_.data.at(address) = (io_data.at(4) << 8) | io_data.at(3);
                    }
                    else
                    {
                        io_state_.data.at(address) = 0;   
                    }
                    break;

                case IODIPU:
                    // TODO
                    io_state_.data.at(address) = 0;
                    break;

                case IODIPD:
                    // TODO
                    io_state_.data.at(address) = 0;
                    break;

                default:
                    io_state_.data.at(address) = 0;
                    break;
            }
        }

        io_state_.mtx.unlock();
        
        // #### UPDATE LINEAR RAIL STATUS ####
        raw_serial_data.clear();
        result = dobot_serial_->getLinearRailStatus(raw_serial_data);
        
        if(result && raw_serial_data.size() > 2)
        {
            is_on_rail_ = (bool)raw_serial_data.at(2);
            // std::cout << is_on_rail_ << std::endl;
        }

        // #### UPDATE LINEAR RAIL POSITION ####
        if(is_on_rail_)
        {
            if(dobot_serial_->getRailPose(raw_serial_data))
            {
                rail_pos_data.y = unpackFloat(raw_serial_data.begin() + 2);

                // std::cout<<rail_pos_data.y<<std::endl;

                current_rail_position_buffer_.mtx.lock();

                current_rail_position_buffer_.pose_data.push_back(rail_pos_data);

                if(current_rail_position_buffer_.pose_data.size() > MAX_BUFFER_SIZE)
                {
                    current_rail_position_buffer_.pose_data.pop_front();
                }

                current_rail_position_buffer_.mtx.unlock();   
                current_rail_position_buffer_.received = true;
            }
        }
        else
        {
            rail_pos_data.y = 0;  

            current_rail_position_buffer_.mtx.lock();

            current_rail_position_buffer_.pose_data.push_back(rail_pos_data);

            if(current_rail_position_buffer_.pose_data.size() > MAX_BUFFER_SIZE)
            {
                current_rail_position_buffer_.pose_data.pop_front();
            }

            current_rail_position_buffer_.mtx.unlock(); 
            current_rail_position_buffer_.received = true;   
        }
        
        // Allow the motor some time for actual update on the hardware. The sampling speed of the
        // encoders are faster than the motion of the motors themselves.

        // TODO : Remove the use of a pausing function in a loop
        std::this_thread::sleep_for(std::chrono::milliseconds(UPDATE_THREAD_DELAY_MS));
    }
}

void DobotStates::updateContinuosPathParams()
{
    std::vector<uint8_t> params_data;
    std::vector<float> cp_params;
    uint8_t real_time_track;

    if(dobot_serial_->getCPParams(params_data))
    {
        if(unpackCPParams(params_data, cp_params, real_time_track))
        {
            cp_params_.plan_accel = cp_params.at(0);
            cp_params_.junction_velocity = cp_params.at(1);
            cp_params_.actual_accel = cp_params.at(2);
            cp_params_.real_time_track = real_time_track;

            cp_params_.received = true;
        }
    }
}

bool DobotStates::initialiseRobot()
{
    dobot_serial_->setQueuedCmdClear();
    dobot_serial_->setQueuedCmdStartExec();

    safety_state_.mtx.lock();
    safety_state_.safety_state = INITIALISING;
    safety_state_.mtx.unlock();

    dobot_serial_->setEMotor(0,false,0);
    dobot_serial_->setEMotor(1,false,0);

    dobot_serial_->setPTPCmd(4,start_joint_angle_); 

    // std::cout<<is_on_rail_<<std::endl;

    dobot_serial_->setLinearRailStatus(is_on_rail_,0,0);  
    dobot_serial_->setHOMECmd();

    std::this_thread::sleep_for(std::chrono::seconds(DOBOT_INIT_TIME));
    
    is_homed_ = true; 

    safety_state_.mtx.lock();
    safety_state_.safety_state = OPERATING;
    safety_state_.mtx.unlock();

    return true;   
}

void DobotStates::setOnRail(bool on_rail)
{
    // std::cout<<on_rail<<std::endl;
    dobot_serial_->setLinearRailStatus(on_rail,0,true);
}

bool DobotStates::getRailStatus()
{
    std::vector<uint8_t> raw_serial_data;

    bool result = dobot_serial_->getLinearRailStatus(raw_serial_data);
    if(result && raw_serial_data.size() > 3)
    {
        std::cout<<raw_serial_data.at(3)<<std::endl;
    }

    return is_on_rail_;
}

void DobotStates::stopAllIO()
{   
    for(int address = 1; address <= 20 ; address ++)
    {
        dobot_serial_->setIOMultiplexing(address,IODummy,false);
    }
}

bool DobotStates::setEStop()
{
    // Stop tool
    bool stop_pump = dobot_serial_->setEndEffectorSuctionCup(0,0,1);
    
    // Stop all current action
    bool stop_queued_cmd = dobot_serial_->setQueuedCmdForceStopExec();
    dobot_serial_->setQueuedCmdClear();

    // Reset all IO ports
    stopAllIO();

    // Reset EMotors
    dobot_serial_->setEMotor(0,false,1);
    dobot_serial_->setEMotor(1,false,1);

    if(stop_pump && stop_queued_cmd)
    {
        is_e_stopped_ = true;
    }
    else 
    {
        is_e_stopped_ = false;
    }

    safety_state_.mtx.lock();
    safety_state_.safety_state = ESTOPPED;
    safety_state_.mtx.unlock();

    return is_e_stopped_;
}

bool DobotStates::setOperate()
{
    if(dobot_serial_->setQueuedCmdStartExec())
    {
        safety_state_.mtx.lock();
        safety_state_.safety_state = OPERATING;
        safety_state_.mtx.unlock();

        return true;
    }
    else return false;
}

bool DobotStates::setPause()
{
    if(dobot_serial_->setQueuedCmdStopExec())
    {
        safety_state_.mtx.lock();
        safety_state_.safety_state = PAUSED;
        safety_state_.mtx.unlock();

        return true;
    }
    else return false;
}

bool DobotStates::setStop()
{
    if(dobot_serial_->setQueuedCmdForceStopExec())
    {
        safety_state_.mtx.lock();
        safety_state_.safety_state = STOPPED;
        safety_state_.mtx.unlock();
        
        return true;
    }
    else return false;
}

SafetyState DobotStates::getRobotSafetyState()
{
    SafetyState current_safety_state;

    safety_state_.mtx.lock();
    current_safety_state = safety_state_.safety_state;
    safety_state_.mtx.unlock();

    return current_safety_state;
}

void DobotStates::resetSafetyState()
{
    is_e_stopped_ = false;
    is_operate_ = false;
    is_paused_ = false;
    is_stopped_ = false;
}

void DobotStates::getIOState(std::vector<int> &io_mux, std::vector<float> &data)
{

    io_state_.mtx.lock();
    
    io_mux = io_state_.io_mux;
    data = io_state_.data;

    io_state_.mtx.unlock();
}