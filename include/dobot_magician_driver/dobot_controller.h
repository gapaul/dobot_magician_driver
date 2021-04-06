#ifndef _DOBOT_CONTROLLER_H_
#define _DOBOT_CONTROLLER_H_

#include <cmath>
#include <vector>
#include <deque>
#include <thread>

#include "dobot_communication.h"
#include "dobot_utils.h"

class DobotController
{
    public:

        DobotController();
        ~DobotController();

        void init(std::shared_ptr<DobotCommunication> dobot_serial_ptr);
        void run();

        void setTargetJointConfiguration(JointConfiguration target_joint_config);
        void setTargetEndEffectorPose(Pose target_end_effector_pose);

        bool moveToTargetJoint();
        bool moveToTargetPose();

        void setToolState(bool state);
        bool getToolState();

        void setIOState(int address, int multiplex, std::vector<double> data);

        // EMotor
        // TODO: convert this to position control instead of velocity control
        void setEMotorSpeed(int index, bool is_enabled, int speed);

        void setTargetRailWithEEPoses(std::vector<double> target_pose);
        bool moveToTargetRailPosition();
        
        // TODO: Trajectory

        // TODO: Velocity control
        void setTargetJointVelocity(JointConfiguration joint_vel_config);
        bool moveWithTargetVelocity();

    private:

        std::shared_ptr<DobotCommunication> dobot_serial_;

        PoseBuffer target_ee_pose_buffer_;
        JointConfigurationBuffer target_joint_config_buffer_;

        JointConfigurationBuffer target_joint_velocity_buffer_;

        UtilsBuffer target_rail_position_buffer_;

        PoseBuffer target_ee_traj_;

        bool tool_state_;

        std::vector<int> direction_;

};

#endif