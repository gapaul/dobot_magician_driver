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
dobot.PublishTargetJoint(joint_target);

%% Publish custom end effector pose
end_effector_position = [0.2,0,0.1];
end_effector_rotation = [0,0,0]
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
    pause(0.02)
end

%% When the robot is in motion, send this
dobot.EStopRobot();

%% Reinitilise Robot
dobot.InitaliseRobot();
