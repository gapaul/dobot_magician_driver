clear all;
clc;
close all;
rosshutdown;
%% Start Dobot Magician Node
rosinit;

%% Start Dobot ROS
dobot = DobotMagician();

%% Test Motion
%% Publish custom joint target
joint_target = [0.0,0.4,0.3,0.0];
dobot.PublishTargetJoint(joint_target);

%% Publish custom end effector pose
end_effector_position = [0.2,0,0.1];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);

%% Turn on tool
dobot.PublishToolState(true);

%% Turn off tool
dobot.PublishToolState(false);

%% Test ESTOP 
%% Send this first
for i = 0.1:0.05:1.0
    joint_target = [0.0,i,0.3,0.0];
    dobot.PublishTargetJoint(joint_target);
    pause(0.1);
end

%% When the robot is in motion, send this
dobot.EStopRobot();

%% Reinitilise Robot
dobot.InitaliseRobot();

%% Set Rail status to true
dobot.SetRobotOnRail(true);

%% Reinitialise robot. It should perform homing with the linear rail
dobot.InitaliseRobot();

%% Move the rail to the position of 0.5
dobot.MoveRailToPosition(0.0);

%% Set Rail status to false
dobot.SetRobotOnRail(false);

%% Reinitialise robot. It should not perform homing with the linear rail
dobot.InitaliseRobot();

%% Test IO
%% Get current IO status of all io pins on the robot
[ioMux, ioData] = dobot.GetCurrentIOStatus();

%% Set a particular pin a particular IO output
address = 1;
ioMux = 1;
data = 1;
dobot.SetIOData(address,ioMux, data);

%% Set particular pin a particular IO input
address = 1;
ioMux = 3;
data = 0;
dobot.SetIOData(address,ioMux,data);

%% Set a velocity to the conveyor belt
conveyorIdx = 1;
enableConveyor = true;
conveyorVelocity = 10000 % Note: this is ticks per second
dobot.SetConveyorBeltVelocity(conveyorIdx,enableConveyor,conveyorVelocity);

%% Turn off conveyor belt
conveyorIdx = 1;
enableConveyor = false;
conveyorVelocity = 0 % Note: this is ticks per second
dobot.SetConveyorBeltVelocity(conveyorIdx,enableConveyor,conveyorVelocity);