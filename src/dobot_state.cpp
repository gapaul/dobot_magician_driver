#include <dobot_magician_driver/dobot_state.h>

DobotStates::DobotStates()
{
    current_joint_config_buffer_.received = false;
    current_end_effector_pose_buffer_.received = false;

    pause_update_thread_ = false;

    cp_params_.received = false;
    cp_params_.user_set = false;

    start_joint_angle_ = {0,0.4,0.3,0};

    // Initialise flag
    is_homed_ = false;
    is_on_rail_ = false;

    is_connected_ = false;
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
    JointConfiguration joint_data;
    JointConfiguration pre_joint_data;

    // std::chrono::duration;

    joint_data.position = std::vector<double>({0,0,0,0});
    joint_data.velocity = std::vector<double>({0,0,0,0});

    auto start_time = std::chrono::system_clock::now();
    auto end_time = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsed_seconds;

    // Check robot connection
    is_connected_ = dobot_serial_->isConnected();
    
    while(true)
    {
        // if(!is_connected_)
        // {
        //     continue;
        // }

        if(dobot_serial_->getPose(raw_serial_data))
        {
            if(unpackPose(raw_serial_data,config_data))
            {
                // UPDATE END EFFECTOR STATE
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
                
                current_end_effector_pose_buffer_.received = true;
                current_end_effector_pose_buffer_.mtx.unlock();


                // UPDATE JOINT STATE
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
                        // std::cout<<duration<<std::endl;
                        joint_data.velocity.at(i) = (joint_data.position.at(i) - pre_joint_data.position.at(i))/duration;
                    }

                    std::cout<<joint_data.velocity.at(0)<<std::endl;
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
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void DobotStates::updateRobotCurrentConfiguration(std::vector<uint8_t> &raw_serial_data, std::vector<double> &config_data, Pose &pose_data, JointConfiguration &joint_data)
{
    // Only update if the connection with the robot is still there   
    // if(!dobot_serial_->isConnected())
    // {
    //     return;
    // }

    // Update robot configuration state
    JointConfiguration pre_joint_data;

    if(dobot_serial_->getPose(raw_serial_data))
    {
        if(unpackPose(raw_serial_data,config_data))
        {
            // Update current end effector pose
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
            
            current_end_effector_pose_buffer_.received = true;
            current_end_effector_pose_buffer_.mtx.unlock();


            // Update current joint configuration
            current_joint_config_buffer_.mtx.lock();
            
            joint_data.position = std::vector<double>(config_data.begin() + 4,config_data.end());

            if(current_joint_config_buffer_.joint_data.size() > 1)
            {
                pre_joint_data = current_joint_config_buffer_.joint_data.back();
                
                for(int i = 0; i < pre_joint_data.position.size(); i ++)
                {

                    double duration = 1;
                    joint_data.velocity.at(i) = (joint_data.position.at(i) - pre_joint_data.position.at(i))/duration;
                }
            }
            
            current_joint_config_buffer_.joint_data.push_back(joint_data);

            if(current_joint_config_buffer_.joint_data.size() > MAX_BUFFER_SIZE)
            {
                current_joint_config_buffer_.joint_data.pop_front();
            }

            current_joint_config_buffer_.received = true;
            // std::cout<<current_joint_config_buffer_.joint_data.back().position.at(0)<<std::endl;
            current_joint_config_buffer_.mtx.unlock();
        }
    }
}

void DobotStates::updateRobotIOStates()
{

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

    dobot_serial_->setEMotor(0,false,0);
    dobot_serial_->setEMotor(1,false,0);

    dobot_serial_->setPTPCmd(4,start_joint_angle_); 

    dobot_serial_->setLinearRailStatus(is_on_rail_,0,0);  
    dobot_serial_->setHOMECmd();
    is_homed_ = true; 

    return true;   
}

void DobotStates::setOnRail(bool on_rail)
{
    is_on_rail_ = on_rail;
    dobot_serial_->setLinearRailStatus(is_on_rail_,0,0);
}