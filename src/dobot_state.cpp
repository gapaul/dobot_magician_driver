#include <dobot_magician_driver/dobot_state.h>

DobotStates::DobotStates()
{
    current_joint_config_buffer_.received = false;
    current_end_effector_pose_buffer_.received = false;

    cp_params_.received = false;
    cp_params_.user_set = false;
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

    while(true)
    {
        updateRobotCurrentConfiguration(raw_serial_data, config_data, pose_data, joint_data);
    }
}

void DobotStates::updateRobotCurrentConfiguration(std::vector<uint8_t> &raw_serial_data, std::vector<double> &config_data, Pose &pose_data, JointConfiguration &joint_data)
{
    // Update robot configuration state

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
            
            current_joint_config_buffer_.joint_data.push_back(joint_data);

            if(current_joint_config_buffer_.joint_data.size() > MAX_BUFFER_SIZE)
            {
                current_joint_config_buffer_.joint_data.pop_front();
            }

            current_joint_config_buffer_.received = true;
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
