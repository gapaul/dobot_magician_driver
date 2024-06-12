#include "dobot_magician_driver/dobot_ros_control.h"

void initialiseDobotHardware(std::shared_ptr<DobotDriver> dobot_driver, std::string port)
{
    // Initialise Dobot communication
    ROS_DEBUG("DobotROSControl: Initialising serial comms module");
    auto dobot_serial = std::make_shared<DobotCommunication>();
    
    ROS_INFO("DobotROSControl: Serial comms initialised. Looking for USB port now (60s timeout)...");
    bool found_port = dobot_serial->init(port);
    ROS_INFO_COND(found_port, "DobotROSControl: Found USB port. Opening port now...", "DobotROSControl: Unable to find USB port. Shutting down...");
    if (!found_port)
    {
        exit(1);
    }
    dobot_serial->startConnection();

    // Initialise state manager
    ROS_DEBUG("DobotROSControl: Initialising State Manager");
    auto dobot_states_manager = std::make_shared<DobotStates>();
    dobot_states_manager->init(dobot_serial);
    dobot_states_manager->run();

    // Initialise controller
    ROS_DEBUG("DobotROSControl: Initialising Controller");
    auto dobot_controller = std::make_shared<DobotController>();
    dobot_controller->init(dobot_serial);

    // Wait for hardware controller and state manager to start updating data
    ROS_DEBUG("DobotROSControl: Allowing State Manager and Controller some time to update data...");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Initialise driver
    ROS_INFO("DobotROSControl: Initialising dobot magician driver");
    dobot_driver->init(dobot_states_manager, dobot_controller);

    // Start updating robot states to ROS network and start control thread
    dobot_driver->run();

    // Initialise robot
    ROS_INFO("DobotROSControl: Initialising ROS wrapper for dobot magician driver");
    ROS_INFO("DobotROSControl: This thread will sleep for Dobot initialise sequence");
    dobot_driver->initialiseRobot();
    ROS_INFO("DobotROSControl: This thread will now wake up");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dobot_magician_ros_control");

    // Set verbosity level to Info
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    std::string port;
    ros::AsyncSpinner spinner(3);
    
    if (!(ros::param::get("~port", port))) 
    {
        ROS_ERROR("DobotROSControl: port not specified");
        exit(1);
    }

    ros::NodeHandle nh;
    ros::NodeHandle pn("~");

    std::shared_ptr<DobotDriver> dobot_driver = std::make_shared<DobotDriver>();

    ROS_INFO("DobotROSControl: Initialising Wrapper. This should block the node until the dobot is ready to receive commands.");
    initialiseDobotHardware(dobot_driver, port);

    ROS_INFO("DobotROSControl: Starting ROS control interface.");
    DobotHardwareInterface dobot_hw_interface(nh, dobot_driver);
    controller_manager::ControllerManager controller_manager(&dobot_hw_interface, nh);

    ros::Rate rate(10); // 50 Hz control loop
    spinner.start();

    while (ros::ok())
    {
        dobot_hw_interface.read();
        controller_manager.update(ros::Time::now(), dobot_hw_interface.getPeriod());
        dobot_hw_interface.write();
        rate.sleep();
    }

    ROS_INFO("DobotROSControl: Shutting down...");
    spinner.stop();
    ros::waitForShutdown();
    return 0;
}
