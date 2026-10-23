%   main_program.m
%   Main program for the control of manipulators
%   Author: Luís Louro, luislouro@algoritmi.uminho.pt
%           Estela Bicho, estela.bicho@dei.uminho.pt
%   % Copyright (C) 2025
%   2025/02/05
%--------------------------------------------------------------------------
% Before running this script, open the scenario in CoppeliaSim, e.g
% Do not run simulation!
%--------------------------------------------------------------------------


function main()
    clear
    % need to choose the arm to control
    robot_name = '/mycobot280';
    ROSWORKING = 0;
    VISIONWORKING = 0;
    
    %Number of targets for each colour of box
    BLUE_CUBE = 1;
    BLUE_CYLINDER = 2;
    RED_CYLINDER = 3;
    TARGET_GREEN = 4;
    
    %----------------------------------------------------------------------
    % Creation of a communication class object with the simulator
    % Try to connect to simulator
    % Input:
    % VISIONWORKING - enable commucation with vision system
    % Output:
    % sim_interface - pointer to class simulator_interface
    % error_sim = 1 - impossible to connect to simulator
    %----------------------------------------------------------------------
    [sim_interface,error_sim] = simulator_interface(VISIONWORKING);
    if(error_sim==1)
        return;
    end
    
    cleanup = onCleanup(@()myCleanupFun(sim_interface));
    
    %----------------------------------------------------------------------
    %Creation of a communication class object with the manipulator arm
    % Input:
    % sim - pointer to class simulator_interface
    % robot_name - name of arm CoppeliaSim model
    % ROSWORKING - enable commucation with real arm using ROS
    % Output:
    % robot_arm - pointer to class arm_interface
    % error_man = 1 - impossible to connect to simulator
    %----------------------------------------------------------------------
    [robot_arm,error_man] = mycobot_interface(sim_interface,robot_name,ROSWORKING);
    if error_man == 1
        sim_interface.terminate();
        return;
    end
    
    sim_interface.start_simulation(); % start simulation in CoppeliaSim
    
    %----------------------------------------------------------------------
    % Informations of arm characteristics
    % Output:
    % nJoints - number of arm joints.
    % Links - dimensions of the links between the axes of rotation
    % MinPositionJoint - array with minimum position for joint 1-6
    % MaxPositionJoint - array with maximum position for joint 1-6
    %----------------------------------------------------------------------
    [nJoints,Links,MinPositionJoint,MaxPositionJoint] = robot_arm.get_RobotCharacteristics();
    
    %Initialize arm with all the joint with 0 degrees
    armJoints(1)=0*pi/180; 
    armJoints(2)=0*pi/180;    
    armJoints(3)=0*pi/180;    
    armJoints(4)=0*pi/180;     
    armJoints(5)=0*pi/180;
    armJoints(6)=0*pi/180;
    %------------------------------------------------------------------
    % Function to send values to arm joints
    % Input:
    % armJoints - array with joint angles (1..4)
    % Output:
    % error - true if joint value outside the limits of the joint
    %-----------------------------------------------------------------
    error = robot_arm.set_joints(armJoints) %send value for arm Joints in rad
    if error==1
        return;
    end

    %------------------------------------------------------------------
    % Functions that allows open the hand
    %------------------------------------------------------------------
    robot_arm.open_hand();      %send Open Hand
    start = tic;
    told=-1
    m=1;
    stop=0;
    sim_interface.step_simulation();
    while stop==0
        %----------------------------------------------------------------------
        % Read Joints function
        % Output:
        % ReadArmJoints - get joint value (rad) for arm
        %_-----------------------------------------------------------------
        ReadArmJoints = robot_arm.get_joints();
        ReadArmJointsDegree = ReadArmJoints * 180/pi; %convert to degrees
    
        %_-----------------------------------------------------------------
        %Read Arm base position
        % Output:
        % armPosition - get robot position
        %_-----------------------------------------------------------------
        armPosition = robot_arm.get_robot_position();
    
        %_-----------------------------------------------------------------
        % Get information from cameraVISIONWORKING
        % Output:
        % image - matrix with image information
        % resolution - resolution information 
        %_-----------------------------------------------------------------
        if VISIONWORKING==1
            [image,resolution] = sim_interface.get_camera_information();
            imshow(image)
            drawnow;
        end

        %_-----------------------------------------------------------------
        % Get Target position information
        % Input:
        % ID of the object (BLUE_CUBE,BLUE_CYLINDER,RED_CYLINDER)
        % Output:
        % targetPosition - get target position
        %_-----------------------------------------------------------------
        %targetPosition=sim_interface.get_target_position(BLUE_CUBE);
        targetPosition=sim_interface.get_target_position(BLUE_CYLINDER);
        %targetPosition=sim_interface.get_target_position(RED_CUBE);
        targetPosition1=sim_interface.get_target_position(TARGET_GREEN);
    
        %----------------------------------------------------------------------
        % --- YOUR CODE --- %
        %-----------------------------------------------------------------
        
        %------------------------------------------------------------------
        % Function to send values to arm joints
        % Input:
        % armJoints - array with joint angles (1..4)
        % Output:
        % error - true if joint value outside the limits of the joint
        %-----------------------------------------------------------------
        %     armJoints(1)=0*pi/180;
        %     armJoints(2)=0*pi/180;
        %     armJoints(3)=0*pi/180;
        %     armJoints(4)=0*pi/180;
        %     armJoints(5)=0*pi/180;
        %     armJoints(6)=0*pi/180;
        %     %------------------------------------------------------------------
        %     % Function to send values to arm joints
        %     % Input:
        %     % armJoints - array with joint angles (1..4)
        %     % Output:
        %     % error - true if joint value outside the limits of the joint
        %     %-----------------------------------------------------------------
        %     error = robot_arm.set_joints(armJoints) %send value for arm Joints in rad
        %     if error==1
        %         return;
        %     end
        % end

        %------------------------------------------------------------------
        % Functions that allows open/close the hand
        %------------------------------------------------------------------
        % robot_arm.open_hand();     %open
        % robot_arm.close_hand();    %close

        m=m+1;
        %%------------- END OF YOUR CODE -------------
        sim_interface.step_simulation();
        time = toc(start);
        if time<0.05
            pause(0.05-time);
        end
        start = tic;
        t = sim_interface.get_simulation_time();
        if t==told
            sim.terminate();
            disp("Simulation stopped");
            return
        end
        told=t;
        fprintf('Simulation time: %.2f [s]\n', t);
        %----------------------------------------------------------------------
    end
    sim_interface.terminate();
    disp('stop simulation')
end

function myCleanupFun(sim_interface)
    disp('stop simulation')
    sim_interface.terminate();
    return
end