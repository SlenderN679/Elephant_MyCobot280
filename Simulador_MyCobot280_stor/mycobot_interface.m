%   mycobot_interface.m
%   Interface to a simulated robot in CoppeliaSim simulator
%   2026/10/07
%   % Copyright (C) 2026
%   Author: Luís Louro, luislouro@algoritmi.uminho.pt
%           Estela Bicho, estela.bicho@dei.uminho.pt

classdef mycobot_interface < handle
    properties (Access = private)
        client
        sim

        RobotHandle  % handle for the robot object
        jointHandle  % 1,2,... handle for each of the joints of the manipulator
        HandHandle   % handle for the hand object (Adaptive Gripper)
        

        ROS_WORKING
        
        
        node_matlab
        sub_joint_position
        pub_joint_single
        msg_joint_single
        pub_joint_group
        msg_joint_group
    end

    properties
        number_of_joints  % variable indicating the number of joints of the manipulator arm
        robot_name        % variable indicating the name of the handler in CoppeliaSim
        MinJointPos       % vector indicating the minimum position allowed for the joint i
        MaxJointPos       % vector indicating the maximun position allowed for the joint i
    end

    methods
        function [obj,error] = mycobot_interface(sim_obj,robot_name,ROSWORKING)
            obj.robot_name = robot_name;
            obj.ROS_WORKING = ROSWORKING;
            error = 0;

            obj.client = sim_obj.get_connection();
            obj.sim = obj.client.require('sim');

            %% Get objects handles

            %Get Arm Handle
            obj.RobotHandle = obj.sim.getObject(robot_name);

            if(obj.RobotHandle==-1)
                disp('ERROR: Failed getting robot handle');
                error = 1;
                return;
            end

            %arm joint handles
            jointName{1} = '/joint1';
            jointName{2} = '/joint2';
            jointName{3} = '/joint3';
            jointName{4} = '/joint4';
            jointName{5} = '/joint5';
            jointName{6} = '/joint6';
            [~,obj.number_of_joints]=size(jointName);
            
            for j=1:obj.number_of_joints
                obj.jointHandle{j}=obj.sim.getObject(jointName{j});
                if(obj.jointHandle{j}==-1)
                    disp('ERROR: Failed getting joint handle');
                    error = 1;
                    return
                end
            end
            

            if obj.ROS_WORKING==1
                %setenv("ROS_DOMAIN_ID", "10");

                %obj.node_matlab = ros2node("/node_matlab");
                %pause(1)
                %obj.sub_joint_position = ros2subscriber(obj.node_matlab,'/px100/joint_states','sensor_msgs/JointState');
                %[obj.pub_joint_single,obj.msg_joint_single] = ros2publisher(obj.node_matlab,'/px100/commands/joint_single','interbotix_xs_msgs/JointSingleCommand');
                %[obj.pub_joint_group,obj.msg_joint_group] = ros2publisher(obj.node_matlab,'/px100/commands/joint_group','interbotix_xs_msgs/JointGroupCommand');

            end

        end

        %Function that allows you to get the arm characteristics
        function [nJoints,Link,MinPositionJoint,MaxPositionJoint] = get_RobotCharacteristics(obj)
            nJoints = obj.number_of_joints;
            MinPositionJoint(1)=-165*pi/180;
            MinPositionJoint(2)=-165*pi/180;
            MinPositionJoint(3)=-165*pi/180;
            MinPositionJoint(4)=-165*pi/180;
            MinPositionJoint(5)=-165*pi/180;
            MinPositionJoint(6)=-175*pi/180;
            MaxPositionJoint(1)=165*pi/180;
            MaxPositionJoint(2)=165*pi/180;
            MaxPositionJoint(3)=165*pi/180;
            MaxPositionJoint(4)=165*pi/180;    
            MaxPositionJoint(5)=165*pi/180;
            MaxPositionJoint(6)=175*pi/180;    
            Link(1)=0.13156;
            Link(2)=0.1104;
            Link(3)=0.096;
            Link(4)=0.07318;
            Link(5)=0.06639;
            Link(6)=0.0436;
            Link(7)=0.115;
            obj.MinJointPos = MinPositionJoint;
            obj.MaxJointPos = MaxPositionJoint;
        end

        %Function that allows you to get the position of the arm
        function armPosition=get_robot_position(obj)
            armPosition_str = obj.sim.getObjectPosition(obj.RobotHandle);
            armPosition(1) = armPosition_str{1};
            armPosition(2) = armPosition_str{2};
            armPosition(3) = armPosition_str{3};
        end

        %Function that allows you to set the joints from the arm
        function error = set_joints(obj, armJoints) %armJoints (1-7) in rad
            error = 0;
            for i=1:1:obj.number_of_joints
                if armJoints(i)<obj.MinJointPos(i) || armJoints(i)>obj.MaxJointPos(i)
                    disp('ERROR: value outside the limits of the joints');
                    error = 1;
                    return;
                else
                    obj.sim.setJointTargetPosition(obj.jointHandle{i},armJoints(i));
                end
            end

            if obj.ROS_WORKING==1
                %obj.msg_joint_group.name = 'arm';
                %obj.msg_joint_group.cmd = [single(armJoints(1)); single(armJoints(2)); single(armJoints(3)); single(armJoints(4))];
                %send(obj.pub_joint_group, obj.msg_joint_group);
            end
        end

        %Function that allows you to get the joints values from the arm
        %armJoints - joint values for arm (in rad)
        function armJoints = get_joints(obj)
            % Get pose
            % x and y in cm
            % phi in rad
            for i = 1:obj.number_of_joints
                %Read from joints
                armJoints(i)=obj.sim.getJointPosition(obj.jointHandle{i});
            end

            % if obj.ROS_WORKING==1
            %     [msg,status,statustext] = receive(obj.sub_joint_position);
            %     armJoints(1)=msg.Position(1);
            %     armJoints(2)=msg.Position(2);
            %     armJoints(3)=msg.Position(3);
            %     armJoints(4)=msg.Position(4);
            %
            % end
        end

        %Functions that allows close the hand
        function close_hand(obj)   
            obj.sim.setInt32Signal('Gripper_open',0);

            if obj.ROS_WORKING==1
                %obj.msg_joint_single.name = 'gripper';
                %obj.msg_joint_single.cmd = single(-200);
                %send(obj.pub_joint_single, obj.msg_joint_single);
            end
            
        end

        %Functions that allows open the hand
        function open_hand(obj)
            obj.sim.setInt32Signal('Gripper_open',1);

            if obj.ROS_WORKING==1
                %obj.msg_joint_single.name = 'gripper';
                %obj.msg_joint_single.cmd = single(200);
                %send(obj.pub_joint_single, obj.msg_joint_single);
            end
        end

        
    end
end
