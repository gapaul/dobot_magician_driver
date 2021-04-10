
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

        void initialiseRobot();
        
        bool isRobotInMotion();
        bool isRobotAtTarget();

        void isRobotOnLinearRail(bool is_on_rail);

        JointConfiguration getCurrentJointConfiguration();
        Pose getCurrentEndEffectorPose();

        Pose getCurrentRailPosition();

        // Single target
        void setTargetJointConfiguration(JointConfiguration target_joint_config);
        void setTargetEndEffectorPose(Pose target_end_effector_pose);

        void setTargetJointConfiguration(std::vector<double> target_joint_config_vec);
        void setTargetEndEffectorPose(std::vector<double> target_end_effector_pose_vec);

        void setTargetRailPosition(double position);
        bool moveToTargetRailPosition();

        bool moveToTargetEndEffectorPose();
        bool moveToTargetJointConfiguration();

        // #TODO: Trajectory

        // Tool state
        void setToolState(bool state);
        bool getToolState();

        // Safety State
        SafetyState getRobotSafetyState();
        void setEStop();
        void setStop();

        void setOperate();

        // EMotor
        void setEMotor(int index, bool is_enabled, int speed);

        // IO 
        void setIOState(int address, int multiplex, std::vector<double> data);
        void getIOState(std::vector<double> &io_mux, std::vector<double> &data);

        // #TODO: Velocity Control
        bool moveWithTargetJointVelocity(std::vector<double> velocity);

    private:

        std::shared_ptr<DobotStates> dobot_state_;
        std::shared_ptr<DobotController> dobot_controller_;

        JointConfiguration current_target_config_;
        Pose current_target_ee_pose_;

        std::thread *driver_update_thread_;

        bool in_motion_;
        bool at_target_;
        bool is_on_rail_;

        void driverUpdateThread();
};  

#endif