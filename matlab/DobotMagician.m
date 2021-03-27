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
   end
   
   properties(Access = private)
   end
   
   methods(Access = public)
       function self = DobotMagician()
           % Initialise subs and pubs as object starts
           self.jointStateSub = rossubscriber('/dobot_magician/joint_states');
           self.endEffectorStateSub = rossubscriber('dobot_magician/end_effector_states');
           
          [self.targetJointTrajPub,self.targetJointTrajMsg] = rospublisher('/dobot_magician/PTP/target_joint_states');
          [self.targetEndEffectorPub,self.targetEndEffectorMsg] = rospublisher('/dobot_magician/PTP/target_end_effector_states');
          
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
   end
   
   methods(Access = private)
       function JointStateCallback(self,src,messsage,~)
           
       end
       
       function EndEffectorStateCallback(self,src,message,~)
           
       end
   end
end