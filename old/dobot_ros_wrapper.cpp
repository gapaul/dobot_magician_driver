#include "dobot_magician_driver/dobot_ros_wrapper.h"

bool DobotRosWrapper::setGripper(dobot_magician_driver::SetEndEffectorRequest &req, dobot_magician_driver::SetEndEffectorResponse &res)
{
    if((req.isEndEffectorEnabled != 0 && req.isEndEffectorEnabled != 1)
            || (req.endEffectorState != 0 && req.endEffectorState != 1)){
        ROS_WARN("DobotRosWrapper: ensure booleans are specified for both request parameters");
        res.success = false;
        return res.success;
    }

    if(_driver->setGripper(req.isEndEffectorEnabled,req.endEffectorState))
    {
        res.success = true;
        return res.success;
    }

    res.success = false;
    return res.success;
}

bool DobotRosWrapper::setSuctionCup(dobot_magician_driver::SetEndEffectorRequest &req, dobot_magician_driver::SetEndEffectorResponse &res)
{
    if((req.isEndEffectorEnabled != 0 && req.isEndEffectorEnabled != 1)
            || (req.endEffectorState != 0 && req.endEffectorState != 1)){
        ROS_WARN("DobotRosWrapper: ensure booleans are specified for both request parameters");
        res.success = false;
        return res.success;
    }

    if(_driver->setSuctionCup(req.isEndEffectorEnabled,req.endEffectorState))
    {
        res.success = true;
        return res.success;
    }

    res.success = false;
    return res.success;
}

bool DobotRosWrapper::setJointAngles(dobot_magician_driver::SetTargetPointsRequest &req, dobot_magician_driver::SetTargetPointsResponse &res)
{
    if(req.target_points.size() != 4){
        ROS_WARN("DobotRosWrapper: specify correct number of target points");
        res.success = false;
        return res.success;
    }

    std::vector<float> target_points;
    for(int i = 0; i < req.target_points.size(); ++i){
        target_points.push_back(req.target_points[i]*180/M_PI);

    }
    if(_driver->setJointAngles(target_points))
    {
        res.success = true;
        return res.success;
    }

    res.success = false;
    return res.success;
}

bool DobotRosWrapper::setCartesianPos(dobot_magician_driver::SetTargetPointsRequest &req, dobot_magician_driver::SetTargetPointsResponse &res)
{
    if(req.target_points.size() != 4){
        ROS_WARN("DobotRosWrapper: specify correct number of target points");
        res.success = false;
        return res.success;
    }

    std::vector<float> target_points;
    for(int i = 0; i < req.target_points.size(); ++i){
        target_points.push_back(req.target_points[i]*1000);
    }

    if(_driver->setCartesianPos(target_points))
    {
        res.success = true;
        return res.success;
    }

    res.success = false;
    return res.success;
}

bool DobotRosWrapper::setCartesianPosWithRail(dobot_magician_driver::SetTargetPointsWithRailRequest &req, dobot_magician_driver::SetTargetPointsWithRailResponse &res)
{
    if (!_driver->isOnLinearRail())
    {
        ROS_WARN("DobotRosWrapper: the linear rail is not initialised");
        res.success = false;
        return res.success;
    }

    if(req.target_points.size() != 5){
        ROS_WARN("DobotRosWrapper: specify correct number of target points");
        res.success = false;
        return res.success;
    }

    std::vector<float> target_points;
    for(int i = 0; i < req.target_points.size(); ++i){
        target_points.push_back(req.target_points[i]*1000);
    }

    if(_driver->setCartesianPosWithRail(target_points))
    {
        res.success = true;
        return res.success;
    }

    res.success = false;
    return res.success;
}

bool DobotRosWrapper::setIOMultiplexing(dobot_magician_driver::SetIOMultiplexingRequest &req, dobot_magician_driver::SetIOMultiplexingResponse &res)
{
    if(!inIORange(req.address)){
        ROS_WARN("DobotRosWrapper: please specify the correct pin address (1 to 20)");
        res.success = false;
        return res.success;
    }

    if(req.multiplex < 1 || req.multiplex > 6){
        ROS_WARN("DobotRosWrapper: please specify the correct multiplex value (1 to 6)");
        res.success = false;
        return res.success;
    }

    if(_driver->setIOMultiplexing(req.address, req.multiplex))
    {
        res.success = true;
        return res.success;
    }

    res.success = false;
    return res.success;
}

bool DobotRosWrapper::setIODigitalOutput(dobot_magician_driver::SetIODigitalOutputRequest &req, dobot_magician_driver::SetIODigitalOutputResponse &res)
{
    int multiplex;
    _driver->getIOMultiplexing(req.address, multiplex);
    if( (IOMux) multiplex != IODO ) {
        ROS_WARN("DobotRosWrapper: the pin is not being mutiplexed for digital output");
        res.success = false;
        return res.success;
    }

    if(!inIORange(req.address)){
        ROS_WARN("DobotRosWrapper: please specify the correct pin address (1 to 20)");
        res.success = false;
        return res.success;
    }

    if(req.level != 0 && req.level != 1){
        ROS_WARN("DobotRosWrapper: ensure to use boolean for the level output (0 or 1)");
        res.success = false;
        return res.success;
    }

    if(_driver->setIODigitalOutput(req.address, req.level))
    {
        res.success = true;
        return res.success;
    }

    res.success = false;
    return res.success;
}

bool DobotRosWrapper::setIOPWMOutput(dobot_magician_driver::SetIOPWMOutputRequest &req, dobot_magician_driver::SetIOPWMOutputResponse &res)
{
    int multiplex;
    _driver->getIOMultiplexing(req.address, multiplex);
    if( (IOMux) multiplex != IOPWM ) {
        ROS_WARN("DobotRosWrapper: the pin is not being mutiplexed for PWM input");
        res.success = false;
        return res.success;
    }

    if(!inIORange(req.address)){
        ROS_WARN("DobotRosWrapper: please specify the correct pin address (1 to 20)");
        res.success = false;
        return res.success;
    }

    if(req.frequency < IO_PWM_HZ_MIN || req.address > IO_PWM_HZ_MAX){
        ROS_WARN("DobotRosWrapper: please specify the correct frequency (10Hz to 1MHz)");
        res.success = false;
        return res.success;
    }

    if(req.duty_cycle < IO_PWM_DC_MIN || req.duty_cycle > IO_PWM_DC_MAX){
        ROS_WARN("DobotRosWrapper: please specify the correct duty cycle (0 to 100)");
        res.success = false;
        return res.success;
    }

    if(_driver->setIOPWM(req.address, req.frequency, req.duty_cycle))
    {
        res.success = true;
        return res.success;
    }

    res.success = false;
    return res.success;
}

bool DobotRosWrapper::getIODigitalInput(dobot_magician_driver::GetIODigitalInputRequest &req, dobot_magician_driver::GetIODigitalInputResponse &res)
{
    int multiplex;
    _driver->getIOMultiplexing(req.address, multiplex);
    if( (IOMux) multiplex != IODI && (IOMux) multiplex != IODIPU && (IOMux) multiplex != IODIPD) {
        ROS_WARN("DobotRosWrapper: the pin is not being mutiplexed for digital input");
        res.success = false;
        return res.success;
    }

    if(!inIORange(req.address)){
        ROS_WARN("DobotRosWrapper: please specify the correct pin address (1 to 20)");
        res.success = false;
        return res.success;
    }

    bool level;
    if(_driver->getIODigitalInput(req.address, level))
    {
        res.level = level;
        res.success = true;
        return res.success;
    }

    res.success = false;
    return res.success;
}

bool DobotRosWrapper::getIOAnalogInput(dobot_magician_driver::GetIOAnalogInputRequest &req, dobot_magician_driver::GetIOAnalogInputResponse &res)
{
    int multiplex;
    _driver->getIOMultiplexing(req.address, multiplex);
    if( (IOMux) multiplex != IOADC ) {
        ROS_WARN("DobotRosWrapper: the pin is not being mutiplexed for analog input");
        res.success = false;
        return res.success;
    }

    if(!inIORange(req.address)){
        ROS_WARN("DobotRosWrapper: please specify the correct pin address (1 to 20)");
        res.success = false;
        return res.success;
    }

    int value;
    if(_driver->getIOAnalogInput(req.address, value))
    {
        res.value = value;
        res.success = true;
        return res.success;
    }

    res.success = false;
    return res.success;
}

bool DobotRosWrapper::setEMotor(dobot_magician_driver::SetEMotorRequest &req, dobot_magician_driver::SetEMotorResponse &res)
{

    if(_driver->setEMotor(req.index,req.is_enabled,req.speed))
    {
        res.success = true;
        return res.success;
    }

    res.success = false;
    return res.success;
}

bool DobotRosWrapper::setCPParams(dobot_magician_driver::SetCPParamsRequest &req, dobot_magician_driver::SetCPParamsResponse &res)
{
    if(req.cp_params.size() != CP_PARAM_SIZE){
        ROS_WARN("DobotRosWrapper: specify correct number of CP parameters");
        res.success = false;
        return res.success;
    }
    if(req.real_time_track != 0 && req.real_time_track != 1){
        ROS_WARN("DobotRosWrapper: ensure to use boolean value only (0 or 1)");
        res.success = false;
        return res.success;
    }

    std::vector<float> cp_params;
    for(int i = 0; i < req.cp_params.size(); ++i){
        cp_params.push_back(req.cp_params[i]*1000);
    }
    if(_driver->setCPParams(cp_params,req.real_time_track))
    {
        res.success = true;
        return res.success;
    }

    res.success = false;
    return res.success;
}

bool DobotRosWrapper::getCPParams(dobot_magician_driver::GetCPParamsRequest &req, dobot_magician_driver::GetCPParamsResponse &res)
{
    std::vector<float> cp_params;
    std::vector<float> converted_cp_params;
    uint8_t real_time_track;

    if(_driver->getCPParams(cp_params,real_time_track))
    {
        for(int i = 0; i < cp_params.size(); ++i)
        {
            converted_cp_params.push_back(cp_params[i]/1000);
        }

        res.cp_params = converted_cp_params;
        res.real_time_track = real_time_track;
        res.success= true;
        return res.success;
    }

    ROS_WARN("Unable to get CPParams. Please try again!");
    res.success = false;
    return res.success;
}

bool DobotRosWrapper::setCPCmd(dobot_magician_driver::SetCPCmdRequest &req, dobot_magician_driver::SetCPCmdResponse &res)
{
    bool is_queued = true;
    if(req.cp_cmd.size() != CP_CMD_SIZE){
        ROS_WARN("DobotRosWrapper: specify correct number of CP points");
        res.success = false;
        return res.success;
    }
    if(req.cp_mode != 0 && req.cp_mode != 1){
        ROS_WARN("DobotRosWrapper: ensure to use boolean value only (0 or 1)");
        res.success = false;
        return res.success;
    }
    std::vector<float> cp_cmd;
    for(int i = 0; i < req.cp_cmd.size(); ++i){
        cp_cmd.push_back(req.cp_cmd[i]*1000);
    }

    if(_driver->setCPCmd(cp_cmd,req.cp_mode))
    {
        res.success = true;
        return res.success;
    }

    res.success = false;
    return res.success;
}

bool DobotRosWrapper::setQueuedCmdStartExec(dobot_magician_driver::SetQueuedCmdRequest &req, dobot_magician_driver::SetQueuedCmdResponse &res)
{
    if (_driver->setQueuedCmdStartExec())
    {
        res.success = true;
        return res.success;
    }
    res.success = false;
    return res.success;
}

bool DobotRosWrapper::setQueuedCmdStopExec(dobot_magician_driver::SetQueuedCmdRequest &req, dobot_magician_driver::SetQueuedCmdResponse &res)
{
    if (_driver->setQueuedCmdStopExec())
    {
        res.success = true;
        return res.success;
    }
    res.success = false;
    return res.success;
}

bool DobotRosWrapper::setQueuedCmdForceStopExec(dobot_magician_driver::SetQueuedCmdRequest &req, dobot_magician_driver::SetQueuedCmdResponse &res)
{
    if (_driver->setQueuedCmdForceStopExec())
    {
        res.success = true;
        return res.success;
    }
    res.success = false;
    return res.success;
}

bool DobotRosWrapper::setQueuedCmdClear(dobot_magician_driver::SetQueuedCmdRequest &req, dobot_magician_driver::SetQueuedCmdResponse &res)
{
    if(_driver->setQueuedCmdClear())
    {
        res.success = true;
        return res.success;
    }
    res.success = false;
    return res.success;
}

bool DobotRosWrapper::setEStop(dobot_magician_driver::SetEStopRequest &req, dobot_magician_driver::SetEStopResponse &res)
{
    if (_driver->setEStop())
    {
        res.success = true;
        return res.success;
    }
    res.success = false;
    return res.success;
}

bool DobotRosWrapper::setInitialise(dobot_magician_driver::SetInitialiseRequest &req, dobot_magician_driver::SetInitialiseResponse &res)
{
    _driver->initialiseDobot();

    ROS_INFO("DobotRosWrapper: this thread will sleep for Dobot initialise sequence");
    if (_driver->isOnLinearRail())
    {
        ROS_INFO("DobotRosWrapper: start Homing with the linear rail");
        std::this_thread::sleep_for(std::chrono::seconds(60));
    }
    else std::this_thread::sleep_for(std::chrono::seconds(30));
    ROS_INFO("DobotRosWrapper: this thread will now wake up");

    res.success = true;
    return res.success;
}

bool DobotRosWrapper::setLinearRailStatus(dobot_magician_driver::SetLinearRailRequest &req, dobot_magician_driver::SetLinearRailResponse &res)
{
    if (_driver->setLinearRailStatus(req.set_linear_rail))
    {
        res.success = true;
        return res.success;
    }
    res.success = false;
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
        _driver->getCurrentConfiguration(latest_cartesian_pos, latest_joint_angles);
        // _driver->getJointAngles(latest_joint_angles);
        // _driver->getCartesianPos(latest_cartesian_pos);

        if(latest_joint_angles.size() == 4 && latest_cartesian_pos.size() == 4)
        {
            joint_ang_msg.position.clear();
            for(int i = 0; i < latest_joint_angles.size(); ++i){
                joint_ang_msg.position.push_back(latest_joint_angles[i]*M_PI/180);
            }

            cart_pos_msg.pose.position.x = latest_cartesian_pos[0]/1000;
            cart_pos_msg.pose.position.y = latest_cartesian_pos[1]/1000;
            cart_pos_msg.pose.position.z = latest_cartesian_pos[2]/1000;

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

bool DobotRosWrapper::inIORange(int address)
{
    if( address < IO_PIN_MIN || address > IO_PIN_MAX ) {return false;}
    else {return true;}
}

DobotRosWrapper::DobotRosWrapper(ros::NodeHandle &nh, ros::NodeHandle &pn, std::string port)
    : _nh(nh)
    , _pn(pn)
    , _rate(100)
{
    _driver = new DobotDriver(port);
    _driver->initialiseDobot();
    _joint_state_pub = _nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    _end_effector_state_pub = _nh.advertise<geometry_msgs::PoseStamped>("end_effector_state", 1);
    _set_gripper_srv = _nh.advertiseService("end_effector/set_gripper", &DobotRosWrapper::setGripper, this);
    _set_suction_cup_srv = _nh.advertiseService("end_effector/set_suction_cup", &DobotRosWrapper::setSuctionCup, this);
    _set_cartesian_pos_srv = _nh.advertiseService("PTP/set_cartesian_pos", &DobotRosWrapper::setCartesianPos, this);
    _set_joint_angles_srv = _nh.advertiseService("PTP/set_joint_angles", &DobotRosWrapper::setJointAngles, this);
    _set_eMotor_srv = _nh.advertiseService("EIO/set_eMotor", &DobotRosWrapper::setEMotor, this);

    _set_io_multiplex_srv = _nh.advertiseService("IO/set_multiplexing", &DobotRosWrapper::setIOMultiplexing, this);
    _set_io_digital_output_srv = _nh.advertiseService("IO/set_digital_output", &DobotRosWrapper::setIODigitalOutput, this);
    _set_io_pwm_output_srv = _nh.advertiseService("IO/set_pwm_output", &DobotRosWrapper::setIOPWMOutput, this);
    _get_io_digital_input_srv = _nh.advertiseService("IO/get_digital_input", &DobotRosWrapper::getIODigitalInput, this);
    _get_io_analog_input_srv = _nh.advertiseService("IO/get_analog_input", &DobotRosWrapper::getIOAnalogInput, this);

    _set_cp_params_srv = _nh.advertiseService("CP/set_cp_params",&DobotRosWrapper::setCPParams,this);
    _get_cp_params_srv = _nh.advertiseService("CP/get_cp_params",&DobotRosWrapper::getCPParams,this);
    _set_cp_cmd_srv = _nh.advertiseService("CP/set_cp_cmd",&DobotRosWrapper::setCPCmd,this);

    _set_queued_cmd_start_srv = _nh.advertiseService("queued_cmd/set_cmd_start_exec",&DobotRosWrapper::setQueuedCmdStartExec,this);
    _set_queued_cmd_stop_srv = _nh.advertiseService("queued_cmd/set_cmd_stop_exec",&DobotRosWrapper::setQueuedCmdStopExec,this);
    _set_queued_cmd_force_stop_srv = _nh.advertiseService("queued_cmd/set_cmd_force_stop_exec",&DobotRosWrapper::setQueuedCmdForceStopExec,this);
    _set_queued_cmd_clear_srv = _nh.advertiseService("queued_cmd/set_clear",&DobotRosWrapper::setQueuedCmdClear,this);

    _set_e_stop_srv = _nh.advertiseService("set_e_stop",&DobotRosWrapper::setEStop,this);

    _set_initialise_srv = _nh.advertiseService("initialise",&DobotRosWrapper::setInitialise,this);

    _set_linear_rail_srv = _nh.advertiseService("linear_rail/set_linear_rail_status",&DobotRosWrapper::setLinearRailStatus,this);
    _set_cartesian_pos_with_rail_srv = _nh.advertiseService("PTP/set_cartesian_pos_with_rail",&DobotRosWrapper::setCartesianPosWithRail,this);

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

//    std::string name = "dobot_magician"+port;

//    std::cout << name << std::endl;

    ros::NodeHandle nh;
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
