#include "dobot_magician_driver/dobot_driver.h"

DobotDriver::DobotDriver()
{
    is_on_rail_ = false;
}

DobotDriver::~DobotDriver()
{
    driver_update_thread_->join();
    delete driver_update_thread_;
}

void DobotDriver::init(std::shared_ptr<DobotStates> dobot_state_ptr, std::shared_ptr<DobotController> dobot_controller_ptr)
{
    dobot_state_ = dobot_state_ptr;
    dobot_controller_ = dobot_controller_ptr;
}

void DobotDriver::run()
{
    driver_update_thread_ = new std::thread(&DobotDriver::driverUpdateThread,this);
}

void DobotDriver::setStateManager(std::shared_ptr<DobotStates> dobot_state_ptr)
{
    dobot_state_ = dobot_state_ptr;
}

void DobotDriver::setController(std::shared_ptr<DobotController> dobot_controller_ptr)
{
    dobot_controller_ = dobot_controller_ptr;
}

void DobotDriver::initialiseRobot()
{
    dobot_controller_->setToolState(false);
    dobot_state_->initialiseRobot();
}

// not a very good way to do this
bool DobotDriver::isRobotAtTarget()
{      
    return at_target_;
}

bool DobotDriver::isRobotInMotion()
{
    return in_motion_;
}

void DobotDriver::isRobotOnLinearRail(bool is_on_rail)
{
    is_on_rail_ = is_on_rail;

    dobot_state_->setOnRail(is_on_rail_);
}

JointConfiguration DobotDriver::getCurrentJointConfiguration()
{
    return dobot_state_->getRobotCurrentJointConfiguration();
}

Pose DobotDriver::getCurrentEndEffectorPose()
{
    return dobot_state_->getRobotCurrentEndEffectorPose();
}

Pose DobotDriver::getCurrentRailPosition()
{
    Pose rail_position = dobot_state_->getCurrentRailPosition();
    rail_position.y = rail_position.y / 1000;
    return rail_position;
}

void DobotDriver::setTargetJointConfiguration(JointConfiguration target_joint_config)
{
    dobot_controller_->setTargetJointConfiguration(target_joint_config);
}

void DobotDriver::setTargetEndEffectorPose(Pose target_end_effector_pose)
{
    dobot_controller_->setTargetEndEffectorPose(target_end_effector_pose);
}

void DobotDriver::setTargetJointConfiguration(std::vector<double> target_joint_config_vec)
{
    JointConfiguration joint_config;
    joint_config.position = target_joint_config_vec;

    dobot_controller_->setTargetJointConfiguration(joint_config);
}

void DobotDriver::setTargetEndEffectorPose(std::vector<double> target_end_effector_pose_vec)
{
    Pose end_effector_pose ;
    end_effector_pose.convert(target_end_effector_pose_vec);
    dobot_controller_->setTargetEndEffectorPose(end_effector_pose);
}

bool DobotDriver::moveToTargetJointConfiguration()
{
    bool result = dobot_controller_->moveToTargetJoint();
    return result;
}

bool DobotDriver::moveToTargetEndEffectorPose()
{
    bool result = dobot_controller_->moveToTargetPose();
    return result;
}

void DobotDriver::driverUpdateThread()
{
    JointConfiguration current_config;
    int check_config = 0;
    
    while(true)
    {
        current_config = dobot_state_->getRobotCurrentJointConfiguration();

        if(current_target_config_.position.size() != current_config.position.size())
        {
            at_target_ = true;
            in_motion_ = false;

            continue;
        }

        for(int i = 0; i < current_config.position.size(); i++)
        {
            
            if(abs(current_config.position.at(i) - current_target_config_.position.at(i)) >= JOINT_ERROR)
            {
                at_target_ = false;
                in_motion_ = true;
            }
            else
            {
                check_config++;
            }
        }

        if(check_config == 4)
        {
            at_target_ = true;
            in_motion_ = false;

            check_config = 0;
        }
        
    }
}

void DobotDriver::setSuctionCupState(bool state)
{
    dobot_controller_->setToolState(state);
}

bool DobotDriver::getToolState()
{
    return dobot_controller_->getToolState();
}

void DobotDriver::setGripperState(bool state, bool enable)
{
    return dobot_controller_->setToolState(state, enable);
}

void DobotDriver::setEStop()
{
    dobot_state_->setEStop();
}

void DobotDriver::setOperate()
{
    dobot_state_->setOperate();
}

void DobotDriver::setStop()
{
    dobot_state_->setStop();
}

SafetyState DobotDriver::getRobotSafetyState()
{
    return dobot_state_->getRobotSafetyState();
}

void DobotDriver::setEMotor(int index, bool is_enabled, int speed)
{
    dobot_controller_->setEMotorSpeed(index,is_enabled,speed);
}

void DobotDriver::setIOState(int address, int multiplex, std::vector<double> data)
{
    dobot_controller_->setIOState(address,multiplex,data);
}

void DobotDriver::getIOState(std::vector<double> &io_mux, std::vector<double> &data)
{
    std::vector<int> out_io;
    std::vector<float> out_data;

    dobot_state_->getIOState(out_io, out_data);

    io_mux = std::vector<double>(out_io.begin(),out_io.end());
    data = std::vector<double>(out_data.begin(),out_data.end());
}

void DobotDriver::setTargetRailPosition(double position)
{
    Pose current_ee_pose = getCurrentEndEffectorPose();

    std::vector<double> pose_with_rail;
    pose_with_rail.push_back(current_ee_pose.x);
    pose_with_rail.push_back(current_ee_pose.y);
    pose_with_rail.push_back(current_ee_pose.z);
    pose_with_rail.push_back(current_ee_pose.theta);

    pose_with_rail.push_back(position);

    dobot_controller_->setTargetRailWithEEPoses(pose_with_rail);
}

bool DobotDriver::moveToTargetRailPosition()
{
    bool result = dobot_controller_->moveToTargetRailPosition();
    return result;
}

bool DobotDriver::moveWithTargetJointVelocity(std::vector<double> velocity)
{
    JointConfiguration joint_vel_accel;
    joint_vel_accel.acceleration = {1,1,1,1};
    joint_vel_accel.velocity = velocity;

    dobot_controller_->setTargetJointVelocity(joint_vel_accel);
    return dobot_controller_->moveWithTargetVelocity();

}