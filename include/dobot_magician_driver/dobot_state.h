#ifndef DOBOT_STATES_H_
#define DOBOT_STATES_H_

#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <thread>

#include "dobot_utils.h"
#include "dobot_communication.h"

class DobotStates
{

    public:

        DobotStates();
        ~DobotStates();

        void init(std::shared_ptr<DobotCommunication> dobot_serial_ptr);
        void run();

        JointConfiguration getRobotCurrentJointConfiguration();
        Pose getRobotCurrentEndEffectorPose();

        void setContinuosPathParams(ContinuousPathParams cp_params);
        ContinuousPathParams getContinousPathParams();

        bool getCurrentConfiguration(std::vector<double> &cart_pos, std::vector<double> &joint_angles);

        bool initialiseRobot();
        void setOnRail(bool on_rail);

    private:

        // Robot connection
        bool is_connected_;

        std::shared_ptr<DobotCommunication> dobot_serial_;

        // Basic states
        JointConfigurationBuffer current_joint_config_buffer_;
        PoseBuffer current_end_effector_pose_buffer_;

        // Dobot Params
        ContinuousPathParams cp_params_;

        // Homing - Initialisation
        bool is_homed_;
        bool is_on_rail_;

        // Others
        std::thread *update_state_thread_;

        bool pause_update_thread_;

        std::vector<float> start_joint_angle_;

        void updateRobotStatesThread();
        
        void updateRobotCurrentConfiguration(std::vector<uint8_t> &raw_serial_data, std::vector<double> &config_data, Pose &pose_data, JointConfiguration &joint_data);
        void updateRobotIOStates();

        void updateContinuosPathParams();

        // Utils functions
        float unpackFloat(std::vector<uint8_t>::iterator it);
        bool unpackPose(std::vector<uint8_t> &data, std::vector<double> &pose);
        bool unpackCPParams(std::vector<uint8_t> &data, std::vector<float> &cp_params, uint8_t &real_time_track);
};

#endif /* DOBOT_STATES_H_ */
