classdef DobotMagician < handle
   properties(Access = private)
        % Subscribers
       jointStateSub;
       endEffectorStateSub;
       
       % Publisher
       targetJointTrajPub;
       targetJointTrajMsg;
       
       targetEndEffectorPub;
       targetEndEffectorMsg;
       
       toolStatePub;
       toolStateMsg;
   end
   
   properties(Access = private)
   end
   
   methods(Access = public)
       function self = DobotMagician()
           % Initialise subs and pubs as object starts

           self.jointStateSub = rossubscriber('/dobot_magician/joint_states');
           self.endEffectorStateSub = rossubscriber('dobot_magician/end_effector_states');
             
           % Publisher for end effector traj control (WIP) For now it can only receive one target pose
          [self.targetJointTrajPub,self.targetJointTrajMsg] = rospublisher('/dobot_magician/PTP/target_joint_states');
          
          % Publisher for joint traj control (WIP) For now it can only receive one single configuration
          [self.targetEndEffectorPub,self.targetEndEffectorMsg] = rospublisher('/dobot_magician/PTP/target_end_effector_states');
          
          [self.toolStatePub, self.toolStateMsg] = rospublisher('/dobot_magician/EE/target_tool_state');
          [self.safetyStatePub,self.safetyStateMsg] = rospublisher('/dobot_magician/Safety/target_safety_state');
          
       end
       
       function publishJointTarget(self, jointTarget)
           trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
           trajectoryPoint.Positions = jointTarget;
           self.targetJointTrajMsg.Points = trajectoryPoint;
           
           send(self.targetJointTrajPub,self.targetJointTrajMsg);
       end
       
       function publishEndEffectorPose(self,pose)
           self.targetEndEffectorMsg.Position.X = pose(1);
           self.targetEndEffectorMsg.Position.Y = pose(2);
           self.targetEndEffectorMsg.Position.Z = pose(3);
           
           qua = eul2quat([0,0,0]);
           self.targetEndEffectorMsg.Orientation.W = qua(1);
           self.targetEndEffectorMsg.Orientation.X = qua(2);
           self.targetEndEffectorMsg.Orientation.Y = qua(3);
           self.targetEndEffectorMsg.Orientation.Z = qua(4);
           
           send(self.targetEndEffectorPub,self.targetEndEffectorMsg);
       end
       
       function publishToolState(self,state)
           self.toolStateMsg.Data = state;
           send(self.toolStatePub,self.toolStateMsg);
       end

       function initaliseRobot(self)
            self.safetyStateMsg.Data = 2; %% Refer to the Dobot Documentation(WIP) - 2 is defined as INITIALISATION 
            send(self.safetyStatePub,self.safetyStateMsg);
       end

       function eStopRobot(self)
            self.safetyStateMsg.Data = 3; %% Refer to the Dobot Documentation(WIP) - 3 is defined as ESTOP 
            send(self.safetyStatePub,self.safetyStateMsg);
       end
   end
   
   methods(Access = private)

       function JointStateCallback(self,src,messsage,~)

       end
       
       function EndEffectorStateCallback(self,src,message,~)
           
       end

       function ToolStateCallback(self,src,message,~)

       end

       function SafetyStatecallback(self,src,message,~)

       end
       
   end
end