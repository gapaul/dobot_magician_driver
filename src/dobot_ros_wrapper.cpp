#include "dobot_magician_driver/dobot_ros_wrapper.h"
#include "dobot_magician_driver/dobot_states.h"



DobotRosWrapper::DobotRosWrapper()
{

}



int main(int argc, char** argv){


    ros::init(argc, argv, "dobot_driver");
    ros::NodeHandle nh;

    DobotStates robot_states;
    DobotCommunication db("/dev/ttyUSB0");

    std::vector<u_int8_t> data;
    db.getPose(data);

    robot_states.unpackPose(data);
//    float x =       db.unpackFloatLE(data.begin());
//    float y =       db.unpackFloatLE(data.begin()+4);
//    float z =       db.unpackFloatLE(data.begin()+8);
//    float r =       db.unpackFloatLE(data.begin()+12);
//    float base =    db.unpackFloatLE(data.begin()+16);
//    float reararm = db.unpackFloatLE(data.begin()+20);
//    float forearm = db.unpackFloatLE(data.begin()+24);
//    float end =     db.unpackFloatLE(data.begin()+28);

    /*
    std::cout << "x: " << x
              << " y: " << y
              << " z: " << z
              << " r: " << r
              << " base: " << base
              << " reararm: " << reararm
              << " forearm: " << forearm
              << " end: " << end
              << std::endl;
    */

    return 0;
}
