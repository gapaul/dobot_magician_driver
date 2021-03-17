#ifndef _DOBOT_DRIVER_H_
#define _DOBOT_DRIVER_H_

#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <thread>

#include "dobot_utils.h"
#include "dobot_state.h"
#include "dobot_controller.h"

class DobotDriver
{
    public:

        DobotDriver();
        ~DobotDriver();

        void init(std::shared_ptr<DobotStates> dobot_state_ptr, std::shared_ptr<DobotController> dobot_controller_ptr);
        void run();

        void setStateManager(std::shared_ptr<DobotStates> dobot_state_ptr);
        void setController(std::shared_ptr<DobotController> dobot_controller_ptr);

        JointConfiguration getCurrentJointConfiguration();
        Pose getCurrentEndEffectorPose();

        void setTargetJointConfiguration(JointConfiguration target_joint_config);
        void setTargetEndEffectorPose(Pose target_end_effector_pose);

        void setTargetJointConfiguration(std::vector<double> target_joint_config_vec);
        void setTargetEndEffectorPose(std::vector<double> target_end_effector_pose_vec);

    private:

        std::shared_ptr<DobotStates> dobot_state_;
        std::shared_ptr<DobotController> dobot_controller_;

        std::thread *update_thread_;

        bool in_motion_;
        bool at_target_;
        
        bool is_e_stopped_;
        bool is_on_rail_;

        void driverUpdateThread();

};  

#endif