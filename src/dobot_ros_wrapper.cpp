#include "dobot_magician_driver/dobot_ros_wrapper.h"

DobotRosWrapper::DobotRosWrapper(ros::NodeHandle &nh, ros::NodeHandle &pn, std::string port)
    : nh_(nh)
    , ph_(pn)
    , rate_(10)
{
    port_ = port;

    // Publishers
    // Robot States
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
    end_effector_state_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("end_effector_poses", 1);
    
    // Tool state
    tool_state_pub_ = nh_.advertise<std_msgs::Bool>("tool_state",1);

    // Safety state
    safety_state_pub_ = nh_.advertise<std_msgs::UInt8>("safety_status",1);

    // Rail Position
    rail_position_pub_ = nh_.advertise<std_msgs::Float64>("rail_position",1);

    // Subscribers
    // Robot control
    target_joint_traj_sub_ = nh_.subscribe("target_joint_states",10,&DobotRosWrapper::jointTargetCallback,this);
    target_end_effector_sub_ = nh_.subscribe("target_end_effector_pose",10,&DobotRosWrapper::endEffectorTargetPoseCallback,this);

    // Tool control
    tool_state_sub_ = nh_.subscribe("target_tool_state",10,&DobotRosWrapper::toolStateCallback,this);

    // Safety state control
    safety_state_sub_ = nh_.subscribe("target_safety_status",10,&DobotRosWrapper::safetyStateCallback,this);

    // Linear Rail control
    use_linear_rail_sub_ = nh_.subscribe("target_rail_status",10,&DobotRosWrapper::linearRailStateCallback,this);
    target_rail_sub_ = nh_.subscribe("target_rail_position",10,&DobotRosWrapper::targetRailPositionCallback,this);

    // EMotor control
    e_motor_sub_ = nh_.subscribe("target_e_motor_state",10,&DobotRosWrapper::eMotorCallback,this);

    // IO control
    io_control_sub_ = nh_.subscribe("target_io_state",10,&DobotRosWrapper::ioStateCallback,this);

    // Use at own risk
    custom_command_sub_ = nh_.subscribe("custom_command",10,&DobotRosWrapper::customCommandCallback,this);

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
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Initialise driver
    dobot_driver_ = std::shared_ptr<DobotDriver>(new DobotDriver);
    dobot_driver_->init(dobot_states_manager_, dobot_controller_);

    // Start updating robot states to ROS network and start control thread
    dobot_driver_->run();

    // Initialise robot
    ROS_INFO("DobotRosWrapper: this thread will sleep for Dobot initialise sequence");
    dobot_driver_->initialiseRobot();
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

    Pose current_rail_position;

    std_msgs::Bool tool_state_msg;
    std_msgs::UInt8 safety_state_msg;
    std_msgs::Float64 rail_pos_msg;
    std_msgs::Float64MultiArray io_state_msg;

    std::vector<double> io_mux;
    std::vector<double> io_data;

    joint_angle_msg.name = joint_names_;

    // Bugged
    // io_state_msg.layout.dim.at(0).label = "height";
    // io_state_msg.layout.dim.at(0).size = 2;
    // io_state_msg.layout.dim.at(0).stride = 40;
    // io_state_msg.layout.dim.at(1).label = "width";
    // io_state_msg.layout.dim.at(1).size = 20;
    // io_state_msg.layout.dim.at(1).stride = 20;

    ROS_INFO("DobotRosWrapper: data from Dobot is now being published");
    ROS_DEBUG("DobotRosWrapper: update_state_thread started");

    ros::Time ros_time;

    bool robot_joints_at_joint_targets = false;
    bool robot_end_effector_at_target_pose = false;

    while(ros::ok())
    {
        joint_angle_msg.position.clear();
        joint_angle_msg.velocity.clear();

        io_state_msg.data.clear();

        io_mux.clear();
        io_data.clear();

        // Joint Configuration
        current_joint = dobot_driver_->getCurrentJointConfiguration();

        for(int i = 0; i < current_joint.position.size(); ++i)
        {
            joint_angle_msg.position.push_back(current_joint.position.at(i)*M_PI/180);
            joint_angle_msg.velocity.push_back(current_joint.velocity.at(i)*M_PI/180);
        }
        
        // End effector Pose
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

        // Rail Position
        current_rail_position = dobot_driver_->getCurrentRailPosition();
        rail_pos_msg.data = current_rail_position.y;

        // Tool state 
        tool_state_msg.data = dobot_driver_->getToolState();

        // Safety state
        safety_state_msg.data = (uint8_t)dobot_driver_->getRobotSafetyState();

        // IO state - Bugged
        // TODO: Find a better way to store these io data
        // dobot_driver_->getIOState(io_mux,io_data);
        
        // for(int i = 0; i < 40; i++)
        // {
        //     if(i < 20)
        //     {
        //         io_state_msg.data.push_back(io_mux.at(i));  
        //     }
        //     else
        //     {
        //         io_state_msg.data.push_back(io_data.at(i-20));  
        //     } 
        // }

        // Update robot state to ROS
        ros_time = ros::Time::now();
        
        // Update joint states
        joint_angle_msg.header.stamp = ros_time;
        joint_state_pub_.publish(joint_angle_msg);

        // Update end effector pose
        cart_pos_msg.header.stamp = ros_time;
        end_effector_state_pub_.publish(cart_pos_msg);

        // Update rail position
        rail_position_pub_.publish(rail_pos_msg);

        // Update tool state
        tool_state_pub_.publish(tool_state_msg);
        
        // // Update safety state
        safety_state_pub_.publish(safety_state_msg);

        // Update IO state
        // io_data_pub_.publish(io_state_msg);
        
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

void DobotRosWrapper::toolStateCallback(const std_msgs::BoolConstPtr& msg)
{
    dobot_driver_->setToolState(msg->data);
}

void DobotRosWrapper::safetyStateCallback(const std_msgs::UInt8ConstPtr& msg)
{
    switch(msg->data)
    {
        case INITIALISING:
            target_joint_data_.received = false;
            target_end_effector_pose_data_.received = false;

            dobot_driver_->initialiseRobot();
            break;

        case ESTOPPED:
            dobot_driver_->setEStop();
            break;
    }
}

void DobotRosWrapper::linearRailStateCallback(const std_msgs::BoolConstPtr& msg)
{
    dobot_driver_->isRobotOnLinearRail(msg->data);
}

void DobotRosWrapper::targetRailPositionCallback(const std_msgs::Float64ConstPtr& msg)
{
    double target_position = msg->data;

    dobot_driver_->setTargetRailPosition(target_position);

    // #TODO: find a way to integrate this into the control loop
    dobot_driver_->moveToTargetJointConfiguration();
}

void DobotRosWrapper::eMotorCallback(const std_msgs::Float64MultiArrayConstPtr& msg)
{
    std::vector<double> data = msg->data;
    if(msg->data.size() < 3)
    {
        dobot_driver_->setEMotor(1,false,0);
        return;
    }
    dobot_driver_->setEMotor((int)msg->data.at(0),(bool)msg->data.at(1),(int)msg->data.at(2));
}

void DobotRosWrapper::ioStateCallback(const std_msgs::Float64MultiArrayConstPtr& msg)
{
    if(msg->data.size() < 3)
    {
        return;
    }

    int address = msg->data.at(0);
    int multiplex = msg->data.at(1);
    std::vector<double> data;

    for(int i = 2; i < msg->data.size() ; i++)
    {
        data.push_back(msg->data.at(i));
    }

    dobot_driver_->setIOState(address, multiplex, data);
}

void DobotRosWrapper::customCommandCallback(const std_msgs::Float64MultiArrayConstPtr& msg)
{
    mtx_.lock();
    custom_command_ = msg->data;

    std::vector<float> converted_command = std::vector<float>(custom_command_.begin(),custom_command_.end());
    mtx_.unlock();

    uint64_t queue_cmd_index = 0;
    dobot_serial_->sendCustomCommand(converted_command,queue_cmd_index,false);    
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
