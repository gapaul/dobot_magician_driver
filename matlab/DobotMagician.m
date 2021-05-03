classdef DobotMagician < handle

   properties(Access = private)
        % Subscribers
       jointStateSub;
       endEffectorStateSub;
       
       ioStatusSub;
       railPosSub;
       
       % Publisher
       targetJointTrajPub;
       targetJointTrajMsg;
       
       targetEndEffectorPub;
       targetEndEffectorMsg;
       
       toolStatePub;
       toolStateMsg;
       
       safetyStatePub;
       safetyStateMsg;
       
       railStatusPub;
       railStatusMsg;
       
       railPosPub;
       railPosMsg;
       
       ioDataPub;
       ioDataMsg;
        
       eMotorPub;
       eMotorMsg;
   end
   
   properties(Access = private)
   end
   
   methods(Access = public)
       function self = DobotMagician()
           % Initialise subs and pubs as object starts

           self.jointStateSub = rossubscriber('/dobot_magician/joint_states');
           self.endEffectorStateSub = rossubscriber('/dobot_magician/end_effector_poses');
           
           self.ioStatusSub = rossubscriber('/dobot_magician/io_data');
           
           % Publisher for joint traj traj control (WIP) For now it can
           % only receive one configuration
          [self.targetJointTrajPub,self.targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
          
          % Publisher for end effector control (WIP) For now it can only
          % receive one target pose
          [self.targetEndEffectorPub,self.targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
          
          [self.toolStatePub, self.toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
          [self.safetyStatePub,self.safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
          
          [self.railStatusPub, self.railStatusMsg] = rospublisher('/dobot_magician/target_rail_status');
          [self.railPosPub,self.railPosMsg] = rospublisher('/dobot_magician/target_rail_position');
          
          [self.ioDataPub,self.ioDataMsg] = rospublisher('/dobot_magician/target_io_state');

          [self.eMotorPub,self.eMotorMsg] = rospublisher('/dobot_magician/target_e_motor_state');
       end
       
       function PublishTargetJoint(self, jointTarget)
           trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
           trajectoryPoint.Positions = jointTarget;
           self.targetJointTrajMsg.Points = trajectoryPoint;
           
           send(self.targetJointTrajPub,self.targetJointTrajMsg);
       end
       
       function PublishEndEffectorPose(self,pose,rotation)
           self.targetEndEffectorMsg.Position.X = pose(1);
           self.targetEndEffectorMsg.Position.Y = pose(2);
           self.targetEndEffectorMsg.Position.Z = pose(3);
           
           qua = eul2quat(rotation);
           self.targetEndEffectorMsg.Orientation.W = qua(1);
           self.targetEndEffectorMsg.Orientation.X = qua(2);
           self.targetEndEffectorMsg.Orientation.Y = qua(3);
           self.targetEndEffectorMsg.Orientation.Z = qua(4);
           
           send(self.targetEndEffectorPub,self.targetEndEffectorMsg);
       end
       
       function PublishToolState(self,state)
           self.toolStateMsg.Data = state;
           send(self.toolStatePub,self.toolStateMsg);
       end

       function InitaliseRobot(self)
            self.safetyStateMsg.Data = 2; %% Refer to the Dobot Documentation(WIP) - 2 is defined as INITIALISATION 
            send(self.safetyStatePub,self.safetyStateMsg);
       end

       function EStopRobot(self)
            self.safetyStateMsg.Data = 3; %% Refer to the Dobot Documentation(WIP) - 3 is defined as ESTOP 
            send(self.safetyStatePub,self.safetyStateMsg);
       end

       function jointStates = GetCurrentJointState(self)
            latestJointStateMsg = self.jointStateSub.LatestMessage;
            jointStates = latestJointStateMsg.Position;
       end
       
       function SetRobotOnRail(self,status)
           self.railStatusMsg.Data = status;
           send(self.railStatusPub,self.railStatusMsg);
       end
       
       function MoveRailToPosition(self,position)
           self.railPosMsg.Data = position;
           send(self.railPosPub,self.railPosMsg);
       end
       
       function [ioMux, ioData] = GetCurrentIOStatus(self)
           latestIODataMsg = self.ioStatusSub.LatestMessage;
           ioStatus = latestIODataMsg.Data;
           ioMux = ioStatus(2:21);
           ioData = ioStatus(23:42);
       end
       
       function SetIOData(self,address,ioMux,data)
           self.ioDataMsg.Data = [address,ioMux,data];
           send(self.ioDataPub,self.ioDataMsg);
       end

       function SetConveyorBeltVelocity(self,enabled,velocity)
            self.eMotorMsg.Data = [enabled,velocity];
            send(self.eMotorPub,self.eMotorMsg);
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