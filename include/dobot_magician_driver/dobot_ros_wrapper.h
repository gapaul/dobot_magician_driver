#include "dobot_magician_driver/dobot_communication.h"
#include "dobot_magician_driver/dobot_states.h"

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"

class DobotRosWrapper {

private:
    ros::Publisher _joint_state_pub;
    ros::Publisher _end_point_state_pub; //is this the best name?
    ros::Publisher _tool_vel_pub; //not sure if exists



    /// thread for constant publishing

public:
    DobotRosWrapper();
};

