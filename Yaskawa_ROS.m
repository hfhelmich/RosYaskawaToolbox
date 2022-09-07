classdef Yaskawa_ROS < matlab.mixin.SetGet
    % This class creates a matlab object that uses the MATLAB ROS toolbox
    % to interface with a running Yaskawa ROS framework.
    
    properties(GetAccess = 'public', SetAccess = 'public')
        jointTrajectory(7,:) double {mustBeReal,mustBeFinite}
    end
    
    properties(GetAccess = 'public', SetAccess = 'private')
        trajPub
        jsSub
        tfSub
        jointNames_ROS = {'joint_s, joint_l, joint_e, joint_u, joint_r, joint_b, joint_t'};
        jointOrder_class = {'joint_s, joint_l, joint_e, joint_u, joint_r, joint_b, joint_t'};
        trajMsg
        homeConfiguration = zeros(7,1);
        jointAngles(7,1) double {mustBeReal,mustBeFinite}
        jointSpeeds(7,1) double {mustBeReal,mustBeFinite}
        jointState(7,2) double {mustBeReal,mustBeFinite}
        robot
        HTform(4,4) double {mustBeReal,mustBeFinite}
        ikin
    end
    
    methods(Access='public')
        function obj = Yaskawa_ROS()
            obj.jointTrajectory = zeros(7,1);   % Home
            obj.trajPub = rospublisher('/ysk/joint_trajectory_MATLAB','IsLatching',false);
            pause(0.25);
            obj.jsSub = rossubscriber('/ysk/joint_states','sensor_msgs/JointState');
            pause(0.25);
            obj.tfSub = rossubscriber('/tf','tf2_msgs/TFMessage');
            pause(0.50);
            obj.jointTrajectory = zeros(7,1);
            tmp = receive(obj.jsSub);
            obj.jointAngles = tmp.Position;
            obj.jointSpeeds = tmp.Velocity;
            obj.jointState = [obj.jointAngles, obj.jointSpeeds];
            
            % Loading urdf data
            obj.robot = importrobot(fullfile('..', 'urdf', 'sia20.urdf'));
            obj.robot.DataFormat = 'column';
            obj.robot.Gravity = [0, 0, -9.81];
            
            % 
            obj.HTform = getTransform(obj.robot, obj.jointTrajectory, "tool0");
            obj.HTform = obj.HTform*Rx(-pi/2)*Ry(pi/2);
            
            % Inverse Kinematics Solver with Parameters
            obj.ikin = inverseKinematics('RigidBodyTree', obj.robot);
            obj.ikin.SolverParameters.AllowRandomRestart = false;
            
            pause(2.0);
            enable_SIA20F
        end
        
        function delete(obj)
            rosshutdown
            clear obj.jsSub obj.trajPub
            pause(2.0)
            disable_SIA20F
            pause(1.0);
            delete(obj);
        end
        
    end
    
    methods
        % Returns 7x1 current joint angles
        function jointAngles = get.jointAngles(obj) % write as callback instead of direct query
            disp('Made it to joint angle call')
            tmp = receive(obj.jsSub, 0.1);
            jointAngles = tmp.Position;
        end
        
        % Returns 7x1 current joint speeds (value returned is almost always
        % zero)
        function jointSpeeds = get.jointSpeeds(obj) % write as callback instead of direct query
            disp('Made it to joint speed call')
            tmp = receive(obj.jsSub, 0.1);
            jointSpeeds = tmp.Velocity;
        end
        
        % Returns 7x2 current joint state (first column is joint angles and
        % second column is joint speeds)
        function jointState = get.jointState(obj) % write as callback instead of direct query
            disp('Made it to joint state call')
            tmp = receive(obj.jsSub, 0.1);
            jointState = [tmp.Position, tmp.Velocity];
        end
        
        
        function goHome(obj)
            % Create joint state command (need only do once)
            msg = rosmessage(obj.trajPub); % creates message of the correct message type
            msg.JointNames = obj.jointNames_ROS; % append names to message
            % msg.JointNames = obj.jointNames; % append names to message
            msg.JointNames = obj.jointNames_ROS; % append names to message
            msg.Points(1) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            msg.Points(1).Positions = obj.homeConfiguration;
            msg.Points(1).Velocities = zeros(7,1);
            msg.Points(1).TimeFromStart.Sec = 5.0;
            %
            %             msg.Points(2) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            %             msg.Points(2).Positions = obj.homeConfiguration;
            %             msg.Points(2).Velocities = zeros(7,1);
            %             msg.Points(2).TimeFromStart.Sec = time;
            %
            send(obj.trajPub, msg)
        end
        
        % FUNCTION: obj.send_jointAngles
        % Purpose:  take one joint angle waypoint and send to the robot
        function msg = send_jointAngles(obj, jAngs)
            msg = rosmessage(obj.trajPub);
            msg.JointNames = obj.jointNames_ROS;
            msg.Points(1) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            msg.Points(1).Positions = jAngs;
            msg.Points(1).Velocities = zeros(7,1);
            % Safe time buffer
            msg.Points(1).TimeFromStart.Sec = 5.0;
            
            send(obj.trajPub,msg);
        end
        
        % FUNCTION: obj.send_taskPos
        % Purpose:  take a single position in task space, convert to joint
        % space, and send to the robot
        % Inputs:
        %   - tPos  - 3x1 matrix (x; y; z)
        function msg = send_taskPos(obj, time, tPos)
            
            obj.HTform = getTransform(obj.robot, obj.jointAngles, "tool0");
            obj.HTform(1:3, 4) = tPos;
            
            [q_sol, q_info] = obj.ikin("tool0", obj.HTform, [0.1, 0.1, 0.1, 10, 0.1, 10],...
                                        obj.jointAngles);
            %             [q, qd] = siaTaskToJoint(obj.jointAngles, obj.ikin, obj.HTform,...
            %                 tPos, [0; 0; 0]);
            %
            %             check = siaCheckLimits(q, 0, qd, 0);
            
            if isequal(q_info.Status, 'success')
                msg = rosmessage(obj.trajPub);
                msg.JointNames = obj.jointNames_ROS;
                msg.Points(1) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
                
                msg.Points(1).Positions = q_sol;
                msg.Points(1).Velocities = zeros(7,1);
                % Safe time buffer
                msg.Points(1).TimeFromStart.Sec = time;
                
                send(obj.trajPub, msg);
            else
                error("Joint angles couldn't be found. Try again.");
            end
        end
        
    end
end