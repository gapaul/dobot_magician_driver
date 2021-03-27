clear all;
clc;
close all;
rosshutdown();
%% Start Dobot Magician Node
rosinit();
% ros.Node('/matlab_node','localhost');

%% Start Dobot ROS
dobot = DobotMagician();

%% Publish custom joint target
for i = 0.1:0.1:1.0
    
    joint_target = [-0.7,0.1,0.3,0.0];

    dobot.publishJointTarget(joint_target);
    pause(1)
    
    joint_target = [0.7,0.1,0.3,0.0];

    dobot.publishJointTarget(joint_target);
    pause(1)
end

%% Publish custom end effector pose

end_effector = [0.2,0,0.1];
dobot.publishEndEffectorPose(end_effector);