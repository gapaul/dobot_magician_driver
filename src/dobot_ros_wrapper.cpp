#include "dobot_magician_driver/dobot_ros_wrapper.h"


bool DobotRosWrapper::setGripper(dobot_magician_driver::SetEndEffectorRequest &req, dobot_magician_driver::SetEndEffectorResponse &res)
{
    if((req.isEndEffectorEnabled != 0 && req.isEndEffectorEnabled != 1)
            || (req.endEffectorState != 0 && req.endEffectorState != 1)){
        ROS_WARN("DobotRosWrapper: ensure booleans are specified for both request parameters");
        return false;
    }


    _driver->dobot_serial->setEndEffectorGripper(req.isEndEffectorEnabled,req.endEffectorState);
    res.success = true;

    return true;
}

bool DobotRosWrapper::setSuctionCup(dobot_magician_driver::SetEndEffectorRequest &req, dobot_magician_driver::SetEndEffectorResponse &res)
{
    if((req.isEndEffectorEnabled != 0 && req.isEndEffectorEnabled != 1)
            || (req.endEffectorState != 0 && req.endEffectorState != 1)){
        ROS_WARN("DobotRosWrapper: ensure booleans are specified for both request parameters");
        return false;
    }

    _driver->dobot_serial->setEndEffectorSuctionCup(req.isEndEffectorEnabled,req.endEffectorState);
    res.success = true;

    return true;

}

bool DobotRosWrapper::setJointAngles(dobot_magician_driver::SetTargetPointsRequest &req, dobot_magician_driver::SetTargetPointsResponse &res)
{
    if(req.target_points.size() != 4){
        ROS_WARN("DobotRosWrapper: specify correct number of target points");
        res.success = false;
        return false;
    }
    std::vector<float> target_points = std::vector<float>(req.target_points.begin(), req.target_points.end());
    _driver->setJointAngles(target_points);
    res.success = true;

    return res.success;

}

bool DobotRosWrapper::setCartesianPos(dobot_magician_driver::SetTargetPointsRequest &req, dobot_magician_driver::SetTargetPointsResponse &res)
{

    if(req.target_points.size() != 4){
        ROS_WARN("DobotRosWrapper: specify correct number of target points");
        return false;
    }
    std::vector<float> target_points = std::vector<float>(req.target_points.begin(), req.target_points.end());
    _driver->setCartesianPos(target_points);
    res.success = true;
    return res.success;


}

void DobotRosWrapper::update_state_loop()
{
    std::vector<double> latest_joint_angles;
    std::vector<double> latest_cartesian_pos;
    sensor_msgs::JointState joint_ang_msg;
    geometry_msgs::PoseStamped cart_pos_msg;

    ROS_INFO("DobotRosWrapper: data from Dobot is now being published");
    ROS_DEBUG("DobotRosWrapper: update_state_thread started");

    ros::Time ros_time;
    while(ros::ok()){
        ros_time = ros::Time::now();
        _driver->getJointAngles(latest_joint_angles);
        _driver->getCartesianPos(latest_cartesian_pos);

        if(latest_joint_angles.size() == 4 && latest_cartesian_pos.size() == 4)
        {
            joint_ang_msg.position.clear();
            for(int i = 0; i < latest_joint_angles.size(); ++i){
                joint_ang_msg.position.push_back(latest_joint_angles[i]);
            }

            cart_pos_msg.pose.position.x = latest_cartesian_pos[0];
            cart_pos_msg.pose.position.y = latest_cartesian_pos[1];
            cart_pos_msg.pose.position.z = latest_cartesian_pos[2];

            if(latest_cartesian_pos[3] < 0.0){
                latest_cartesian_pos[3] = latest_cartesian_pos[3]+360.0;
            }

            cart_pos_msg.pose.orientation.w = (cos(latest_cartesian_pos[3]*0.5*M_PI/180.0)); //assumes only changes in yaw
            cart_pos_msg.pose.orientation.x = 0;
            cart_pos_msg.pose.orientation.y = 0;
            cart_pos_msg.pose.orientation.z = sqrt(1-pow(cart_pos_msg.pose.orientation.w,2));


            cart_pos_msg.header.stamp = ros_time;
            joint_ang_msg.header.stamp = ros_time;
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
    _driver->initialiseDobot();
    _joint_state_pub = _nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
    _end_effector_state_pub = _nh.advertise<geometry_msgs::PoseStamped>("/end_effector_state", 1);
    _set_gripper_srv = _nh.advertiseService("/end_effector/set_gripper", &DobotRosWrapper::setGripper, this);
    _set_suction_cup_srv = _nh.advertiseService("/end_effector/set_suction_cup", &DobotRosWrapper::setSuctionCup, this);
    _set_cartesian_pos_srv = _nh.advertiseService("/PTP/set_cartesian_pos", &DobotRosWrapper::setCartesianPos, this);
    _set_joint_angles_srv = _nh.advertiseService("/PTP/set_joint_angles", &DobotRosWrapper::setJointAngles, this);

    ROS_INFO("DobotRosWrapper: this thread will sleep for Dobot initialise sequence");
    std::this_thread::sleep_for(std::chrono::seconds(30));
    ROS_INFO("DobotRosWrapper: this thread will now wake up");


    update_state_thread = new std::thread(&DobotRosWrapper::update_state_loop, this);

}

DobotRosWrapper::~DobotRosWrapper()
{
    update_state_thread->join();

}



int main(int argc, char** argv){


    ros::init(argc, argv, "dobot_magician_node");
    std::string port;
    ros::AsyncSpinner spinner(3);
    if (!(ros::param::get("~port", port))) {

        ROS_ERROR("DobotRosWrapper: port not specified");
        exit(1);

    }

    std::string name = "dobot_magician"+port;

//    std::cout << name << std::endl;

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
//           std::cout<< "waiting for keypress" << std::endl;

//        std::cin >> temp;
//        db_ros.setJointAngles(vect);
        rate.sleep();
        //        ros::spinOnce(); //async
    }
//    while(!ros::shutdown());
    spinner.stop();
    return 0;
}
