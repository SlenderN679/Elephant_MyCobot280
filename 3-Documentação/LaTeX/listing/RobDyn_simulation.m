classdef RobDyn_simulation < handle
    %SAMU_SIMULATION V-REP connection interface for a SAMU scene
    %   The class provides the communication service and information 
    %   pre-processing for a MATLAB based control interface with a
    %   simulated SAMU project developed robot in VREP.
    %   The several settings and options are explained in situ.
    %   2018/04/12
    %   % Copyright (C) 2021, by Luís Louro
    %   Author: Luís Louro, llouro@dei.uminho.pt
    
    properties (Access = private)
        vrep
        clientID
    end
    
    properties
    end
    
    methods
        % Create a connection handler to a SAMU Simulation scene
        % remote_ip_address The remote IP address
        % remote_port The remote port
        function [obj,error] = RobDyn_simulation(remote_ip_address, remote_port)
            error = 0;
            ip_address = '127.0.0.1';
            port = 19997;
            if nargin >= 1
                % TODO: Add safe guarding for well formated ip
                ip_address = remote_ip_address;
            end
            if nargin >= 2
                % TODO: Check for valid number
                port = remote_port;
            end
            
            obj.vrep = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
            obj.vrep.simxFinish(-1); % just in case, close all opened connections

            obj.clientID = obj.vrep.simxStart(ip_address, port, true, true, 5000, 5);
                       
            if (obj.clientID > -1)
                fprintf('\n');
                disp('INFO: Connected successfully to CoppeliaSim!');
                fprintf('\n');
                
            else
                clear obj;
                disp('ERROR: Failed connecting to remote API server. Ensure CoppeliaSim is running and a scenario is loaded.');
                error = 1;
                return;
            end
            
            %% Synchronous mode
            obj.vrep.simxSynchronous(obj.clientID, true); % Enable the synchronous mode 
            obj.vrep.simxStartSimulation(obj.clientID, obj.vrep.simx_opmode_oneshot);
            
            obj.vrep.simxSynchronousTrigger(obj.clientID); % Trigger next simulation step (Blocking function call)
            
           % The first simulation step is now being executed
           %ensure it is finished so we can access signals
            obj.vrep.simxGetPingTime(obj.clientID);
                        
            %% Setup data streaming
            % simulation time
            [res, ~] = obj.vrep.simxGetFloatSignal(obj.clientID,'SimulationTime', obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                msg = 'ERROR: Failed to setup data streaming for simulation time';
                error(msg);
            end           
        end
        
        function [vrep, clientID] = get_connection(obj)
            vrep = obj.vrep;
            clientID = obj.clientID;
        end
        
        % call before setting velocity and getting data
        function ensure_all_data(obj)
            obj.vrep.simxGetPingTime(obj.clientID);
        end
        
        % call just after all data have been collected.
        function trigger_simulation(obj)
            obj.vrep.simxSynchronousTrigger(obj.clientID);
        end
                
        function [error,sim_time] = get_simulation_time(obj)
            error = 0;
            [res, sim_time] = obj.vrep.simxGetFloatSignal(obj.clientID, 'SimulationTime', obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok )
                disp('ERROR: The simulation stopped in CoppeliaSim');
                error = 1;
                obj.terminate();
                return;
            end
        end
        
        function [error,sim_timestep] = get_simulation_timestep(obj)
            error = 0;
            [res, sim_timestep] = obj.vrep.simxGetFloatSignal(obj.clientID, 'SimulationTimeStep', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok )
                disp('ERROR: Failed getting simulation time step!');
                error = 1;
                obj.terminate();
                return;
            end
        end
        
        function error = terminate(obj)
            error = 0;
            res = obj.vrep.simxStopSimulation(obj.clientID, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed stopping simulation!');
                error = 1;
                return;
            end
            obj.vrep.simxGetPingTime(obj.clientID);

            % Now close the connection to V-REP:    
            obj.vrep.simxFinish(obj.clientID);

            obj.vrep.delete(); % call the destructor!
        end
    end
    
end

