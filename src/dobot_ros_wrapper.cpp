#include "dobot_magician_driver/dobot_ros_wrapper.h"


bool DobotRosWrapper::setSuctionCup(dobot_magician_driver::SetSuctionCupRequest &req, dobot_magician_driver::SetSuctionCupResponse &res)
{
    if(!req.pumpOn){
        req.pumpOn = false;
    }

    res.success =_driver->dobot_serial->setEndEffectorSuctionCup(1,req.pumpOn);

    return res.success;

}

bool DobotRosWrapper::setJointAngles(dobot_magician_driver::SetTargetPointsRequest &req, dobot_magician_driver::SetTargetPointsResponse &res)
{
    std::cout << req.target_points.size() << std::endl;
    if(req.target_points.size() != 4){
        ROS_INFO("DobotRosWrapper: specify correct number of target points");
        res.success = false;
        return false;
    }
    std::vector<float> test = std::vector<float>(req.target_points.begin(), req.target_points.end());
    res.success =_driver->setJointAngles(test);

    return res.success;

}

bool DobotRosWrapper::setCartesianPos(dobot_magician_driver::SetTargetPointsRequest &req, dobot_magician_driver::SetTargetPointsResponse &res)
{

    if(req.target_points.size() != 4){
        ROS_INFO("DobotRosWrapper: specify correct number of target points");
        return false;
    }
    std::vector<float> test = std::vector<float>(req.target_points.begin(), req.target_points.end());
    res.success =_driver->setCartesianPos(test);

    return res.success;


}

void DobotRosWrapper::update_state_loop()
{
    std::vector<double> latest_joint_angles;
    std::vector<double> latest_cartesian_pos;
    sensor_msgs::JointState joint_ang_msg;
    geometry_msgs::Pose cart_pos_msg;

    ROS_DEBUG("DobotRosWrapper: update_state_thread started");

    while(ros::ok()){

        _driver->getJointAngles(latest_joint_angles);
        _driver->getCartesianPos(latest_cartesian_pos);

        // needs fixing, it was hacked together
        if(latest_joint_angles.size() == 4 && latest_cartesian_pos.size() == 4)
        {
            joint_ang_msg.position.clear();
            for(int i = 0; i < latest_joint_angles.size(); ++i){

                joint_ang_msg.position.push_back(latest_joint_angles[i]);
            }

            cart_pos_msg.position.x = latest_cartesian_pos[0];
            cart_pos_msg.position.y = latest_cartesian_pos[1];
            cart_pos_msg.position.z = latest_cartesian_pos[2];
//            cart_pos_msg.orientation.

            _joint_state_pub.publish(joint_ang_msg);
            _end_effector_state_pub.publish(cart_pos_msg);
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
    _joint_state_pub = _pn.advertise<sensor_msgs::JointState>("/joint_states", 1);
    _end_effector_state_pub = _pn.advertise<geometry_msgs::Pose>("/end_effector_state", 1);
    _set_suction_cup_srv = _nh.advertiseService("/end_effector/set_suction_cup", &DobotRosWrapper::setSuctionCup, this);
    _set_cartesian_pos_srv = _nh.advertiseService("/PTP/set_cartesian_pos", &DobotRosWrapper::setCartesianPos, this);
    _set_joint_angles_srv = _nh.advertiseService("/PTP/set_joint_angles", &DobotRosWrapper::setJointAngles, this);

    std::this_thread::sleep_for(std::chrono::seconds(1));

    update_state_thread = new std::thread(&DobotRosWrapper::update_state_loop, this);

}

DobotRosWrapper::~DobotRosWrapper()
{
    update_state_thread->join();

}



int main(int argc, char** argv){


    ros::init(argc, argv, "dobot_magician_node");


    std::string port;
    ros::AsyncSpinner spinner(4);
    if (!(ros::param::get("~port", port))) {

        ROS_ERROR("DobotRosWrapper: port not specified");
        exit(1);

    }

    std::string name = "dobot_magician"+port;

    std::cout << name << std::endl;

    ros::NodeHandle nh(name); // remove "/dev", seems too long
    ros::NodeHandle pn("~");
    DobotRosWrapper db_ros(nh,pn,port);


    // For want of something better to add, "please use ROS Logger for all logging"
    // // http://wiki.ros.org/roscpp/Overview/Logging
    // #include <ros/console.h>
    // ROS_DEBUG("Hello %s", "World");
    // ROS_DEBUG_STREAM("Hello " << "World");
    //    std::cout << "ASDFGHJK" << std::endl;
    ROS_DEBUG("DobotRosWrapper: spinner.start()");
    spinner.start();
    ros::Rate rate(10);
    while(ros::ok()){
        //        ros::spinOnce(); //async
        rate.sleep();
    }
//    while(!ros::shutdown());
    spinner.stop();
    return 0;
}
