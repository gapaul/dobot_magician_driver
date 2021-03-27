#include "dobot_magician_driver/dobot_ros_wrapper.h"

DobotRosWrapper::DobotRosWrapper(ros::NodeHandle &nh, ros::NodeHandle &pn, std::string port)
    : nh_(nh)
    , ph_(pn)
    , rate_(10)
{
    port_ = port;

    // Publisher
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
    end_effector_state_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("end_effector_states", 1);

    // Subscriber
    target_joint_traj_sub_ = nh_.subscribe("PTP/target_joint_states",10,&DobotRosWrapper::jointTargetCallback,this);
    target_end_effector_sub_ = nh_.subscribe("PTP/target_end_effector_states",10,&DobotRosWrapper::endEffectorTargetPoseCallback,this);

    target_joint_data_.received = false;
    target_end_effector_pose_data_.received = false;
}

DobotRosWrapper::~DobotRosWrapper()
{
    update_state_thread_->join();
    robot_control_thread_->join();
}

void DobotRosWrapper::init()
{
    // Establish communication with hardware
    dobot_serial_ = std::shared_ptr<DobotCommunication>(new DobotCommunication());
    dobot_serial_->init(port_);
    dobot_serial_->startConnection();

    // Initialise state manager
    dobot_states_manager_ = std::shared_ptr<DobotStates>(new DobotStates);
    dobot_states_manager_->init(dobot_serial_);
    dobot_states_manager_->run();

    // Initialise controller
    dobot_controller_ = std::shared_ptr<DobotController>(new DobotController);
    dobot_controller_->init(dobot_serial_);

    // Wait for hardware controller and state manager to start updating data
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Initialise driver
    dobot_driver_ = std::shared_ptr<DobotDriver>(new DobotDriver);
    dobot_driver_->init(dobot_states_manager_, dobot_controller_);

    // Start updating robot states to ROS network and start control thread
    dobot_driver_->run();

    // Initialise robot
    // dobot_driver_->initialiseRobot();
    ROS_INFO("DobotRosWrapper: this thread will sleep for Dobot initialise sequence");
    // std::this_thread::sleep_for(std::chrono::seconds(30));
    ROS_INFO("DobotRosWrapper: this thread will now wake up");
}

void DobotRosWrapper::run()
{
    update_state_thread_ = new std::thread(&DobotRosWrapper::updateStateThread, this);
    robot_control_thread_ = new std::thread(&DobotRosWrapper::robotControlThread, this);
}

void DobotRosWrapper::updateStateThread()
{
    sensor_msgs::JointState joint_angle_msg;
    geometry_msgs::PoseStamped cart_pos_msg;

    JointConfiguration current_joint;
    Pose current_ee_pose;

    ROS_INFO("DobotRosWrapper: data from Dobot is now being published");
    ROS_DEBUG("DobotRosWrapper: update_state_thread started");

    ros::Time ros_time;

    bool robot_joints_at_joint_targets = false;
    bool robot_end_effector_at_target_pose = false;

    while(ros::ok())
    {
        joint_angle_msg.position.clear();
        joint_angle_msg.velocity.clear();

        ros_time = ros::Time::now();
        current_joint = dobot_driver_->getCurrentJointConfiguration();

        for(int i = 0; i < current_joint.position.size(); ++i)
        {
            joint_angle_msg.position.push_back(current_joint.position.at(i)*M_PI/180);
            joint_angle_msg.velocity.push_back(current_joint.velocity.at(i)*M_PI/180);
        }
        
        current_ee_pose = dobot_driver_->getCurrentEndEffectorPose();

        cart_pos_msg.pose.position.x = current_ee_pose.x/1000;
        cart_pos_msg.pose.position.y = current_ee_pose.y/1000;
        cart_pos_msg.pose.position.z = current_ee_pose.z/1000;

        if(current_ee_pose.theta < 0.0)
        {
            current_ee_pose.theta = current_ee_pose.theta + 360.0;
        }
        
        cart_pos_msg.pose.orientation.w = (cos(current_ee_pose.theta*0.5*M_PI/180.0)); //assumes only changes in yaw
        cart_pos_msg.pose.orientation.x = 0;
        cart_pos_msg.pose.orientation.y = 0;
        cart_pos_msg.pose.orientation.z = sqrt(1-pow(cart_pos_msg.pose.orientation.w,2));

        // Update robot state to ROS
        joint_angle_msg.header.stamp = ros_time;
        joint_angle_msg.name = joint_names_;
        joint_state_pub_.publish(joint_angle_msg);

        cart_pos_msg.header.stamp = ros_time;
        end_effector_state_pub_.publish(cart_pos_msg);

        rate_.sleep();
    }
}

void DobotRosWrapper::robotControlThread()
{
    ROS_INFO("DobotRosWrapper: Control thread started.");

    while(ros::ok())
    {
        // Control loop
        // If receives target joints and robot is not moving, move joints
        if(target_joint_data_.received)
        {
            bool result = moveToTargetJoints();
            // std::cout<<"Result: " << result << std::endl;
            target_joint_data_.received = false;
        }
        // If receives target pose and robot is not moving, move to pose
        if(target_end_effector_pose_data_.received)
        {
            bool result = moveToTargetEndEffectorPose();
            // std::cout<<"Result: " << result << std::endl;
            target_end_effector_pose_data_.received = false;
        }

        rate_.sleep();
    }
}

void DobotRosWrapper::jointTargetCallback(const trajectory_msgs::JointTrajectoryConstPtr& msg)
{
    ROS_INFO("New target joint configurations received !!");

    JointConfiguration target_joint;

    target_joint_data_.mtx.lock();
    // target_joint_data_.joint_data.positions = msg->points.at(0).positions;
    
    for(int i = 0; i < 4; i++)
    {
        target_joint.position.push_back(msg->points.at(0).positions.at(i) * 180 / M_PI);
    }  
    
    target_joint_data_.mtx.unlock();

    dobot_driver_->setTargetJointConfiguration(target_joint);

    target_joint_data_.received = true;
}

void DobotRosWrapper::endEffectorTargetPoseCallback(const geometry_msgs::PoseConstPtr& msg)
{
    ROS_INFO("New target end effector pose received !!");

    Pose target_pose;
    
    target_end_effector_pose_data_.mtx.lock();
    // target_end_effector_pose_data_.pose_data.position = msg->position;

    target_pose.x = msg->position.x * 1000;
    target_pose.y = msg->position.y * 1000;
    target_pose.z = msg->position.z * 1000;
    target_pose.theta = tf::getYaw(msg->orientation);

    target_end_effector_pose_data_.mtx.unlock();

    dobot_driver_->setTargetEndEffectorPose(target_pose);
    
    target_end_effector_pose_data_.received = true;
}

bool DobotRosWrapper::moveToTargetJoints()
{
    bool result = dobot_driver_->moveToTargetJointConfiguration();
    return result;
}

bool DobotRosWrapper::moveToTargetEndEffectorPose()
{
    bool result = dobot_driver_->moveToTargetEndEffectorPose();
    return result;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dobot_magician_node");
    std::string port;
    ros::AsyncSpinner spinner(3);
    
    if (!(ros::param::get("~port", port))) 
    {
        ROS_ERROR("DobotRosWrapper: port not specified");
        exit(1);
    }

    ros::NodeHandle nh;
    ros::NodeHandle pn("~");

    DobotRosWrapper db_ros(nh,pn,port);
    
    ROS_INFO("Initialising Wrapper.");
    db_ros.init();

    ROS_INFO("Starting Wrapper.");
    db_ros.run();

    spinner.start();
    ros::Rate rate(10);
    while(ros::ok)
    {
        rate.sleep();
    }
    spinner.stop();
    ros::waitForShutdown();
    return 0;
}
