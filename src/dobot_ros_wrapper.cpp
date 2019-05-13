#include "dobot_magician_driver/dobot_ros_wrapper.h"





bool DobotRosWrapper::setSuctionCup(dobot_magician_driver::SetSuctionCupRequest &req, dobot_magician_driver::SetSuctionCupResponse &res)
{


}

void DobotRosWrapper::update_state_loop()
{
    std::vector<double> latest_joint_angles;
    sensor_msgs::JointState msg;

    ROS_DEBUG("DobotRosWrapper: update_state_thread started");

    while(ros::ok()){

        _driver->getJointAngles(latest_joint_angles);

        if(latest_joint_angles.size() == 4)
        {
            msg.position.clear();
            for(int i = 0; i < latest_joint_angles.size(); ++i){

                msg.position.push_back(latest_joint_angles[i]);
            }

            _joint_state_pub.publish(msg);
        }

        _rate.sleep();
    }

}


DobotRosWrapper::DobotRosWrapper(ros::NodeHandle &nh, ros::NodeHandle &pn, std::string port)
    : _nh(nh)
    , _pn(pn)
    , _rate(100)
{
    _driver = new DobotDriver(port);
    _joint_state_pub = _nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
    _set_suction_cup_srv = _nh.advertiseService("/end_effector/set_suction_cup", &DobotRosWrapper::setSuctionCup, this);

    update_state_thread = new std::thread(&DobotRosWrapper::update_state_loop, this);
}

DobotRosWrapper::~DobotRosWrapper()
{
    update_state_thread->join();

}



int main(int argc, char** argv){


    ros::init(argc, argv, "dobot_driver");
    ros::NodeHandle nh;
    ros::NodeHandle pn;


    DobotRosWrapper db_ros(nh,pn,"/dev/ttyUSB0");



   // For want of something better to add, "please use ROS Logger for all logging"
   // // http://wiki.ros.org/roscpp/Overview/Logging
   // #include <ros/console.h>
   // ROS_DEBUG("Hello %s", "World");
   // ROS_DEBUG_STREAM("Hello " << "World");
    

    ros::Rate rate(10);
    while(ros::ok()){

        ros::spinOnce(); //async
        rate.sleep();
    }

    return 0;
}
