clear all;
clc;
close all;
rosshutdown;
%% Start Dobot Magician Node
rosinit;

%% Start Dobot ROS
dobot = DobotMagician();

%% Publish custom joint target
joint_target = [0.0,0.4,0.3,0.0];
dobot.publishJointTarget(joint_target);

%% Publish custom end effector pose
end_effector = [0.2,0,0.1];
dobot.publishEndEffectorPose(end_effector);

%% Turn on tool
dobot.publishToolState(true);

%% Turn off tool
dobot.publishToolState(false);

%% Test ESTOP 
%% Send this first
for i = 0.1:0.05:1.0
    joint_target = [0.0,i,0.3,0.0];
    dobot.publishJointTarget(joint_target);
    pause(0.02)
end

%% When the robot is in motion, send this
dobot.eStopRobot();

%% Reinitilise Robot
dobot.initaliseRobot();
