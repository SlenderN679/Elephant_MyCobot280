classdef RobDyn_robot < handle
    %RobDyn_ROBOT Interface to a simulated robot in CoppeliaSim simulator
    %   Detailed explanation goes here
    
    properties (Access = private)
        vrep
        clientID
        
        RobotHandle
        steering_motor_handle
        TargetHandle
        sensorHandle
        LeftMotorHandle
        RightMotorHandle
  
        world_ref_handle
        
    end
    
    properties
        name
        TARGET_Number
        Rrobot
        Obst
        NsensorsIRobs
        Dwheels
        RadiusWheels
    end
    
    methods
        function [obj,error] = RobDyn_robot(sim_obj)
            
            error = 0;
            
            obj.TARGET_Number=2;
            
            obj.Rrobot = 48.0/2;
            
            obj.Dwheels = 0.32;    %m
            
            obj.RadiusWheels = 0.1/2;    %m
            
            obj.NsensorsIRobs = 12; 
            
            [obj.vrep, obj.clientID] = sim_obj.get_connection();

                       
            if (obj.clientID > -1)
                disp('INFO: Connected successfully to CoppeliaSim!');
            else
                clear obj;
                disp('ERROR: Failed connecting to remote API server. Ensure that the simulation is running in CoppeliaSim.');
                error = 1;
            end
            
            %% Get objects handles
            obj.world_ref_handle = -1;
            
            obj_name = 'RobotDyn';
            [res, obj.RobotHandle] = obj.vrep.simxGetObjectHandle(obj.clientID, obj_name, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                disp('ERROR: Failed getting robot handle ');
                error = 1;
                return;
            end
            
            [res,~] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.RobotHandle,-1,obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed getting robot position information');
                error = 1;
                return;
            end

            [res,~] = obj.vrep.simxGetObjectOrientation(obj.clientID,obj.RobotHandle,-1,obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed getting robot orientation information');
                error = 1;
                return;
            end
            
            % target handle

            for t=1:obj.TARGET_Number
                targetName{t} = ['Target',num2str(t)];
                [res,obj.TargetHandle{t}]=obj.vrep.simxGetObjectHandle(obj.clientID,targetName{t},obj.vrep.simx_opmode_blocking);

                if (res ~= obj.vrep.simx_return_ok)
                    disp('ERROR: Failed getting target handle ');
                    error = 1;
                    return;
                end
                
                [res,~] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.TargetHandle{t},-1,obj.vrep.simx_opmode_streaming);
                if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                    disp('ERROR: Failed getting target position information');
                    error = 1;
                    return;
                end
            end


            %sensor handles
            m=1;

            for j=1:obj.NsensorsIRobs

                sensorName{j} = ['RobotDyn_s',num2str(j)];

                [res,obj.sensorHandle{m}]=obj.vrep.simxGetObjectHandle(obj.clientID,sensorName{j},obj.vrep.simx_opmode_blocking);

                if (res ~= obj.vrep.simx_return_ok)
                    disp('ERROR: Failed getting sensor handle ');
                    error = 1;
                    return;
                end
                [res,~,~,~,~]=obj.vrep.simxReadProximitySensor(obj.clientID, obj.sensorHandle{m},obj.vrep.simx_opmode_streaming);

                if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                    disp('ERROR: Failed getting sensor data information');
                    error = 1;
                    return;
                end

                m=m+1;
            end

            % robot handle for motors
            [res,obj.LeftMotorHandle]=obj.vrep.simxGetObjectHandle(obj.clientID,'RobotDyn_lMotor',obj.vrep.simx_opmode_blocking);

            if (res ~= obj.vrep.simx_return_ok)
                disp('ERROR: Failed getting Left Motor handle ');
                error = 1;
                return;
            end

            [res,obj.RightMotorHandle]=obj.vrep.simxGetObjectHandle(obj.clientID,'RobotDyn_rMotor',obj.vrep.simx_opmode_blocking);

            if (res ~= obj.vrep.simx_return_ok)
                disp('ERROR: Failed getting Right Motor handle ');
                error = 1;
                return;
            end
        end
        function [error, Rrobot,Obst] = get_RobotCharacteristics(obj)
            error = 0;
            Rrobot = obj.Rrobot;                
            distance_between_sensors = 22.5*pi/180;   %rad

            for i=1:obj.NsensorsIRobs
                Obst(i) = ((-fix(obj.NsensorsIRobs/2.0) + i - 1.0) * distance_between_sensors);
            end
        end
        
        function  [error, x, y, phi] = set_velocity(obj, wrobot, vrobot) %wrobot - rad/s; vrobot - cm/s
            %% Set vehicle velocity    
            error = 0;
            dv = wrobot*obj.Dwheels/2.0;  	%m/s , Wrobot = (vl-vr)/Dist_between_the_wheels*/
            vLeft = vrobot*0.01-dv;      	%m/s*/
            vRight = vrobot*0.01+dv;        %m/s*/
            
            wLeft = vLeft/obj.RadiusWheels;
            res = obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.LeftMotorHandle, wLeft, obj.vrep.simx_opmode_oneshot );
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed sending vLeft data!');
                error = 1;
                return;
            end
            
            wRight = vRight/obj.RadiusWheels;
            res = obj.vrep.simxSetJointTargetVelocity(obj.clientID, obj.RightMotorHandle, wRight, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed sending vRight data!');
                error = 1;
                return;
            end
            
            % Get pose
            [error,x, y, phi] = get_vehicle_pose(obj);
            if(error==1)
                return;
            end
            
        end
        
        function [error, x, y, phi] = get_vehicle_pose(obj)
            % Get pose
            % x and y in cm
            % phi in rad
            error = 0;
            [res, robot_pos] = obj.vrep.simxGetObjectPosition(obj.clientID, obj.RobotHandle, -1, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('Failed getting robot position!');
                error = 1;
                return;
            end
            
            [res, robot_ori] = obj.vrep.simxGetObjectOrientation(obj.clientID, obj.RobotHandle, -1, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('Failed getting robot orientation!');
                error = 1;
                return;
            end
            
            x = robot_pos(1)*100;   %cm
            y = robot_pos(2)*100;   %cm
            phi = robot_ori(3);     %rad/s
        end
        
         function [error, x, y, phi] = get_vehicle_pose2pi(obj)
             error = 0;
             [error, x, y, phi] = get_vehicle_pose(obj);
             if(error==0)
                phi = rem(phi + 2*pi, 2*pi);
             end
         end
         
         function [error,XTARGET,YTARGET] = get_TargetPosition( obj, t)
            %UNTITLED2 Summary of this function goes here
            %   Detailed explanation goes here
            %XTARGET and YTARGET in cm
            error = 0;
            [res,tposition] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.TargetHandle{t},-1,obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed getting target position!');
                error = 1;
                return;
            end

            XTARGET=tposition(1)*100;      %cm
            YTARGET=tposition(2)*100;      %cm
         end
            
         function [error,dc] = get_DistanceSensorAquisition( obj )
            %UNTITLED Summary of this function goes here
            %   Detailed explanation goes here
            %dc in cm
            error = 0;
            dc = zeros(1,obj.NsensorsIRobs);

            for s=1:obj.NsensorsIRobs,
                [res,detectionState,detectedPoint,~,~]=obj.vrep.simxReadProximitySensor(obj.clientID, obj.sensorHandle{s},obj.vrep.simx_opmode_buffer);

                if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                    disp('ERROR: Failed getting sensor data information');
                    error = 1;
                    return;
                end

                if detectionState==0
                    dc(s)=80;       %cm
                else
                    dc(s)=detectedPoint(3)*100; %cm
                end
            end

        end
        
        
    end
    
end

