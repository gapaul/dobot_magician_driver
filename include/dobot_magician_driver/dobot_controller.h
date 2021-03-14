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

    private:

        std::shared_ptr<DobotCommunication> dobot_serial_;

        PoseBuffer target_ee_pose_buffer_;
        JointConfigurationBuffer target_joint_config_buffer_;

};

#endif