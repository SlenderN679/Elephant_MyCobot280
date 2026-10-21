%   simulator_interface.m
%   Interface to simulator in CoppeliaSim simulator
%   2023/03/01
%   % Copyright (C) 2023
%   Author: Luís Louro, llouro@dei.uminho.pt
%           Estela Bicho, estela.bicho@dei.uminho.pt

classdef simulator_interface < handle

    properties (Access = private)
        client
        sim
        simVision
        TargetHandle        % handle for the target object (8 different)
        TargetNames         % list of target names
        CameraHandle        % handler for camera

        VISION_WORKING
    end

    properties
        TARGET_Number       % Number of targets

    end

    methods
        % Create a connection handler to a SAMU Simulation scene
        % remote_ip_address The remote IP address
        % remote_port The remote port
        function [obj,error] = simulator_interface(VISIONWORKING)
            error = 0;
            obj.client = RemoteAPIClient();
            obj.sim = obj.client.require('sim');
            obj.simVision = obj.client.require('simVision');

            obj.VISION_WORKING = VISIONWORKING;


            % Four different boxes
            %obj.TARGET_Number = 3;
            obj.TargetNames{1} = '/BlueCube';
            obj.TargetNames{2} = '/BlueCylinder';
            obj.TargetNames{3} = '/RedCube';
            obj.TargetNames{4} = '/TargetGreen';
            obj.TargetNames{5} = '/Block1';
            obj.TargetNames{6} = '/Block2';
            obj.TargetNames{7} = '/Block3';

            [~,obj.TARGET_Number]=size(obj.TargetNames);

            %Get Targets Handle
            for a=1:obj.TARGET_Number
                obj.TargetHandle{a} = obj.sim.getObject(obj.TargetNames{a});
                if(obj.TargetHandle{a}==-1)
                    disp('ERROR: Failed getting target handle');
                    error = 1;
                    return;
                end
            end

            if obj.VISION_WORKING==1
                obj.CameraHandle = obj.sim.getObject('/Vision_sensor');
                if (obj.CameraHandle==-1)
                    disp('ERROR: Failed getting camera handle');
                    error = 1;
                    return;
                end
                 [ imageData, res] = obj.sim.getVisionSensorImg(obj.CameraHandle);
            end
        end

        function start_simulation(obj)
            obj.sim.setStepping(true);
            obj.sim.startSimulation();
        end

        function step_simulation(obj)
            obj.sim.step();
        end
        
        function client = get_connection(obj)
            client = obj.client;
        end

        %Function that allows you to get the position of the target
        function targetPosition=get_target_position(obj,iTarget)
            targetPosition_cell = obj.sim.getObjectPosition(obj.TargetHandle{iTarget});
            targetPosition(1)=targetPosition_cell{1};      %m
            targetPosition(2)=targetPosition_cell{2};      %m
            targetPosition(3)=targetPosition_cell{3};      %m
        end
%% ======================================================================
% Função que devolve a orientação de um alvo
%======================================================================
        % Saída: targetOrientation = [alfa beta gama] (rad, ângulos de Euler)
        %        gama é a rotação em torno de z para objetos pousados na mesa
        function targetOrientation=get_target_orientation(obj,iTarget)
            o = obj.sim.getObjectOrientation(obj.TargetHandle{iTarget}, -1);
            targetOrientation = [o{1} o{2} o{3}];
        end
%% ======================================================================
% /Configuração do exemplo pick-and-place/
%======================================================================

        %Function that allows you to get the joints values from the arm
        %armJoints - joint values for arm (in rad)
        function [image,resolution] = get_camera_information(obj)
            if obj.VISION_WORKING==1
                [ imageData, res] = obj.sim.getVisionSensorImg(obj.CameraHandle);
                width = res{1};
                height = res{2};
                image = flip(permute(reshape(imageData, 3, width, height), [3 2 1]), 1);
                resolution = [width height];
            end
        end

        function t = get_simulation_time(obj)
            t = obj.sim.getSimulationTime();
        end
        
        function sim_timestep = get_simulation_timestep(obj)
            sim_timestep = obj.sim.getFloatSignal('SimulationTimeStep');
        end
        
        function terminate(obj)
            obj.sim.stopSimulation();
            %obj.sim.delete(); % call the destructor!
        end
    end
end