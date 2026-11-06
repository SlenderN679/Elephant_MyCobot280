%% IMPORTANT: 
% Before running this script, open the scenario in CoppeliaSim, e.g
% Do not run simulation!
%
%% Last change: Estela Bicho Erlhagen, 18/10/2018


% Try to connect to simulator
[sim, error] = RobDyn_simulation();
if(error==1)
    return;
end


[vehicle2,error2] = RobDyn_robot2(sim);
[vehicle,error] = RobDyn_robot(sim);
%initialize robot class
% error = 1 - error in function
if(error==1)
    return;
end
if(error2==1)
    return;
end

[error,timestep] = sim.get_simulation_timestep();

%get time step value (normally dt=50ms)
% error = 1 - error in function
if(error==1)
    return;
end

[error,Rrobot,theta_obs] = vehicle.get_RobotCharacteristics();
%Rrobot - robot radius (in cm)
%theta_obs - Vector with angle value for each sensor i (i = 1, . . . , 11) relative to the
%frontal direction (in rad)
%error = 1 - error in function
if(error==1)
    return;
end

%------------------------------------------------------------------------------
%for future
%param = dyn_tar_parameters(timestep,Rrobot,theta_obs);
%param = dyn_obs_parameters(timestep,Rrobot,theta_obs);
%param = dyn_obstar_parameters(timestep,Rrobot,theta_obs);
%------------------------------------------------------------------------------

% Set initial velocity vrobot = 0 (linear velocity) and wrobot = 0 (angular velocity)
% and get robot's inicial pose
wrobot = 0; %rad/s
vrobot = 0; %cm/s

[error, xrobot, yrobot, phirobot] = vehicle.set_velocity(wrobot, vrobot);
% robot  (xrobot,yrobot) in cm
% phirobot - rad
% wrobot - rad/s
% vrobot - cm/s
% error = 1 - error in function

% Define the (first) Target
ntarget=1;

% if(error2==1)
%     return;
% end

% [error2,timestep2] = vehicle2.get_simulation_timestep();
% 
% if(error2==1)
%     return;
% end

[error2,Rrobot2,theta_obs2] = vehicle2.get_RobotCharacteristics();

if(error2==1)
    return;
end

wrobot2 = 0; %rad/s
vrobot2 = 0; %cm/s

[error2, xrobot2, yrobot2, phirobot2] = vehicle2.set_velocity(wrobot2, vrobot2);

if(error2==1)
    return;
end
%%%---------------------- Start Robot Motion Behavior -------------------
%%%-----------------------------------------------------------------------
while ntarget<=vehicle.TARGET_Number % until robot goes to target ntarget
    %% Robot interface
   % set and get information to/from vrep
   % avoid do processing in between ensure_all_data and trigger_simulation
   sim.ensure_all_data();
   % set robot angular velocity (rad/s) and robot linear velocity (cm/s)
   % get also pose. it can be used vehicle.get_vehicle_pose() instead
   [error,xrobot, yrobot, phirobot] = vehicle.set_velocity(wrobot, vrobot);
   % robot position (xrobot,yrobot) in cm
   % phirobot - rad
   % wrobot - rad/s
   % vrobot - cm/s
   % error = 1 - error in function
   if(error==1)
       return;
   end
   
   %Get distance from obstacles (11 infrared obstacles - distance between
   %0-80cm)
   [error,dc] = vehicle.get_DistanceSensorAquisition();  %distance in cm
   
   % error = 1 - error in function
   if(error==1)
       return;
   end
   
   %Get target position for target 1 or 2 (t)
   [error,XTARGET,YTARGET] = vehicle.get_TargetPosition(ntarget);
   %position (XTARGET,YTARGET) in cm
   % error = 1 - error in function
   if(error==1)
       return;
   end
  
   %get simulation time
   [error,sim_time] = sim.get_simulation_time();
   % error = 1 - error in function
   if(error==1)
        return;
   end
   
   %trigger simulation step
   sim.trigger_simulation();

  
  
   %% Processing step
   % Implement the dynamic systems that govern vehicle's behavior and
   % compute new angular and linear velocities...
   % The simulation timestep, dt, is stored in timestep (value is not changed
   % while simulation is running)
   % the simulation time is stored in sim_time.
   
   %%----------- BEGIN YOUR CODE HERE ----------- %
   %Set parameter values
   Tau_tar=10*timestep;
   lambda_tar=1/Tau_tar;
   N=length(dc);
   dtheta_obs=theta_obs(2)-theta_obs(1);
%    dtheta_obs= 2*pi/N ;%gama angular dos setores
   beta1=22.5;
   beta2=20;
   too_far=80;
   
   %Create vectores for psi_obs,lambda_obs, sigma_obs
   Q=0.01; %magnitude stochastic force
  
   psi_obs=zeros(N,1);
   lambda_obs=zeros(N,1);
   sigma_obs=zeros(N,1);
   V_pot=zeros(N,1);
   
   %compute the contribution
   f_obs=zeros(N,1);
   F_obs=0.0;
   U = 0;
   
%    for i=2:1:(N-1)
   for i=1:N
   % compute repeller value
      psi_obs(i)=  theta_obs(i)+phirobot;

      %compute magnitude of repulsion
      if(dc(i) < too_far)
        lambda_obs(i)=beta1*exp(-dc(i)/beta2);
%       else
%           lambda_obs(i)=0;
      end 
      %compute of range of repulsion
      sigma_obs(i)=atan(tan(dtheta_obs/2)+Rrobot/(Rrobot+dc(i)));
      
      %compute f_obs_i
      f_obs(i)=lambda_obs(i)*(phirobot-psi_obs(i))*exp(-(phirobot-psi_obs(i))^2/(2*(sigma_obs(i)^2)));
      
%       V_pot(i)= lambda_obs(i)*(sigma_obs(i)^2)*exp(-(((phirobot-psi_obs(i))^2)/(2*(sigma_obs(i))^2)))-(lambda_obs(i)*((sigma_obs(i)^2)/sqrt(exp(1))))*0.1;
      V_pot(i)= lambda_obs(i)*(sigma_obs(i)^2)*exp(-(((phirobot-psi_obs(i))^2)/(2*(sigma_obs(i))^2)))-lambda_obs(i)*sigma_obs(i)^2/sqrt(exp(1))*0.05;
     U = U + V_pot(i);
     F_obs=F_obs+f_obs(i);%+V_pot(i);
     
   end
   
   
   %compute psi_tar
   psi_tar=atan2((YTARGET-yrobot),(XTARGET-xrobot));
   %erro=phirobot-psi_tar;
%    erro=phirobot-psi_tar;
   
   %compute ftar
   ftar=-lambda_tar*sin(phirobot-psi_tar);
   
   %compute stochastic force
   fstoch=sqrt(Q)*randn(1,1);
   
   %compute total force
   f_total=ftar+F_obs+fstoch;
   
   %compute angular velocity
   wrobot=f_total;
  
   distance = sqrt((xrobot-XTARGET)^2+(yrobot-YTARGET)^2);
   %set linear velocity
   
   alpha = atan(2*U)/pi;
  
   vrobot = 50*(1-alpha);

   if(distance < 3.5*Rrobot)
       if (ntarget == 1)
           ntarget = 2;
       else
           ntarget = 1;
       end
   end
%  ----------------------------------------------
%  ---------------- VEHICLE 2 -------------------
%    % set robot angular velocity (rad/s) and robot linear velocity (cm/s)
%    % get also pose. it can be used vehicle.get_vehicle_pose() instead
%    [error2,xrobot2, yrobot2, phirobot2] = vehicle2.set_velocity(wrobot2, vrobot2);
%    % robot position (xrobot,yrobot) in cm
%    % phirobot - rad
%    % wrobot - rad/s
%    % vrobot - cm/s
%    % error = 1 - error in function
%    if(error2==1)
%        return;
%    end
%    
%    %Get distance from obstacles (11 infrared obstacles - distance between
%    %0-80cm)
%    [error2,dc2] = vehicle2.get_DistanceSensorAquisition();  %distance in cm
%    
%    % error = 1 - error in function
%    if(error2==1)
%        return;
%    end
%    
%    %Get target position for target 1 or 2 (t)
%    %[error2,XTARGET2,YTARGET2] = vehicle2.get_TargetPosition(ntarget);
%    XTARGET2 = xrobot;
%    YTARGET2 = yrobot;
%    %position (XTARGET,YTARGET) in cm
%    % error = 1 - error in function
%    if(error2==1)
%        return;
%    end
% 
%    psi_obs2=zeros(N,1);
%    lambda_obs2=zeros(N,1);
%    sigma_obs2=zeros(N,1);
%    V_pot2=zeros(N,1);
%    
%    %compute the contribution
%    f_obs2=zeros(N,1);
%    F_obs2=0.0;
%    U2 = 0;
% %    for i=2:1:(N-1)
%    for i=1:N
%    % compute repeller value
%       psi_obs2(i)=  theta_obs2(i)+phirobot2;
% 
%       %compute magnitude of repulsion
%       if(dc2(i) < too_far)
%         lambda_obs2(i)=beta1*exp(-dc2(i)/beta2);
% %       else
% %           lambda_obs(i)=0;
%       end 
%       %compute of range of repulsion
%       sigma_obs2(i)=atan(tan(dtheta_obs/2)+Rrobot2/(Rrobot2+dc2(i)));
%       
%       %compute f_obs_i
%       f_obs2(i)=lambda_obs2(i)*(phirobot2-psi_obs2(i))*exp(-(phirobot2-psi_obs2(i))^2/(2*(sigma_obs2(i)^2)));
%       
% %       V_pot(i)= lambda_obs(i)*(sigma_obs(i)^2)*exp(-(((phirobot-psi_obs(i))^2)/(2*(sigma_obs(i))^2)))-(lambda_obs(i)*((sigma_obs(i)^2)/sqrt(exp(1))))*0.1;
%       V_pot2(i)= lambda_obs2(i)*(sigma_obs2(i)^2)*exp(-(((phirobot2-psi_obs2(i))^2)/(2*(sigma_obs2(i))^2)))-lambda_obs2(i)*sigma_obs2(i)^2/sqrt(exp(1))*0.05;
%      U2 = U2 + V_pot2(i);
%      F_obs2=F_obs2+f_obs2(i);%+V_pot(i);
%      
%    end
%    
%    
%    %compute psi_tar
%    psi_tar2=atan2((YTARGET2-yrobot2),(XTARGET2-xrobot2));
%    %erro=phirobot-psi_tar;
% %    erro=phirobot-psi_tar;
%    
%    %compute ftar
%    ftar2=-lambda_tar*sin(phirobot2-psi_tar2);
%    
%    %compute stochastic force
%    fstoch2=sqrt(Q)*randn(1,1);
%    
%    %compute total force
%    f_total2=ftar2+F_obs2+fstoch2;
%    
%    %compute angular velocity
%    wrobot2=f_total2;
%   
%    distance2 = sqrt((xrobot2-XTARGET2)^2+(yrobot2-YTARGET2)^2);
%    %set linear velocity
%    
%    alpha2 = atan(2*U2)/pi;
%   
%    vrobot2 = 50*(1-alpha2);
%     
%     if(distance2 < 2.5*Rrobot2)
%         vrobot2 = 0;
%     end

%GRAFICOS
   x1=zeros(360,N);
   Vpot_plot=zeros(360,1);
   f_obs_plot=zeros(360,1);
   n=zeros(360,1);
   n1=zeros(360,1);
   x=zeros(360,1);
   aux_obs=zeros(360,1);

   for i=1:360
    n(i)=((2*pi)/180)*i;
    n1(i)=i;
    %x(i)=sin(n(i)-psi_tar);
    x(i)=-lambda_tar*sin(n(i)-psi_tar);
    for j=2:1:(N-1)
        
      % compute repeller value
      psi_obs(j)=phirobot + theta_obs(j);
      
      sigma_obs(j)=atan(tan(dtheta_obs/2)+Rrobot/(Rrobot+dc(j)));
      %compute magnitude of repulsion
      if(dc(j)) < too_far
          lambda_obs(j)=beta1*exp(-dc(j)/beta2);
      end
      %compute of range of repulsion
      
      
      %compute f_obs_i
      f_obs(i,j)=lambda_obs(j)*(n(i)-psi_obs(j))*exp(-((n(i)-psi_obs(j))^2)/2*(sigma_obs(j))^2);
      %f_obs(i,j)=lambda_obs(j)*(phirobot-psi_obs(j))*exp(-((phirobot-psi_obs(j))^2)/2*(sigma_obs(j))^2);
      f_obs_plot(i)=f_obs_plot(i)+f_obs(i,j);
      Vpot_plot(i)= Vpot_plot(i)+lambda_obs(j)*(sigma_obs(j)^2)*exp(-(((n(i)-psi_obs(j))^2)/(2*(sigma_obs(j))^2)))-(lambda_obs(j)*((sigma_obs(j)^2)/sqrt(exp(1))))*0.01;
           
    end
   aux_obs(i)=f_obs_plot(i);        %fobs
   f_obs_plot(i)=aux_obs(i)+x(i);   %total fobs+ falvo (x(i))
   end
   
   
   figure(1)
   set(gcf, 'Position', get(0,'Screensize')); % Fullscreen Plot
   subplot(3,1,1) %falvo
   plot(n1,x)
   %plot(n,x); %o de cima ou este?
   axis([1 360 -2 2])
   xline(phirobot,'-','Robot')
   xlabel('?')
   ylabel('d?/dt')
   
 %  figure(2)
   subplot(3,1,2)   %ftotal
   plot(n1,x1,n1,f_obs_plot,'.',n1,x,'+',n1,aux_obs,'*')
   axis([0 360 -15 15])
   xline(((phirobot+pi)/pi)*180,'-','Robot')
   xlabel('?')
   ylabel('d?/dt') 
 %  figure(3)
   subplot(3,1,3)
   plot(n1,aux_obs,n1,Vpot_plot) %total e potencial
   axis([0 360 -15 15])
   xline(phirobot+pi,'--r','Target')
   xlabel('?')
   ylabel('d?/dt')

%%----------------------------------------------------------------------------------
%     vehicle2.ensure_all_data();
%    [error2,xrobot2,yrobot2,phirobot2] = vehicle2.set_velocity(wrobot2, vrobot2);
%    
%    if(error2==1)
%        return;
%    end
%    
%    [error2,dc2] = vehicle2.get_DistanceSensorAquisition();
%    
%    if(error2==1)
%        return;
%    end
%    
%    [error2,sim_time2] = vehicle2.get_simulation_time();
%    XTARGET2 = xrobot;
%    YTARGET2 = yrobot;
%    
%       
%    psi_obs2=zeros(N,1);
%    lambda_obs2=zeros(N,1);
%    sigma_obs2=zeros(N,1);
%    V_pot2=zeros(N,1);   
%    f_obs2=zeros(N,1);
%    F_obs2=0.0;
%    U2 = 0;
%    
%    for i=1:N
%     psi_obs2(i)=  theta_obs2(i)+phirobot2;
%    
%     if(dc2(i) < too_far)
%         lambda_obs2(i)=beta1*exp(-dc2(i)/beta2);
%     end
%    
%     sigma_obs2(i)=atan(tan(dtheta_obs/2)+Rrobot2/(Rrobot2+dc2(i)));
% 
%     f_obs2(i)=lambda_obs2(i)*(phirobot2-psi_obs2(i))*exp(-(phirobot2-psi_obs2(i))^2/(2*(sigma_obs2(i)^2)));
%    
%     V_pot2(i)= lambda_obs2(i)*(sigma_obs2(i)^2)*exp(-(((phirobot2-psi_obs2(i))^2)/(2*(sigma_obs2(i))^2)))-lambda_obs2(i)*sigma_obs2(i)^2/sqrt(exp(1))*0.05;
%     U2 = U2 + V_pot2(i);
%     F_obs2=F_obs2+f_obs2(i);
%    end
%    
%    psi_tar2=atan2((YTARGET2-yrobot2),(XTARGET2-xrobot2));
%    ftar2=-lambda_tar*sin(phirobot2-psi_tar2);
%    
%    f_total2=ftar2+F_obs2+fstoch;
%    wrobot2=f_total2;
%    distance2 = sqrt((xrobot2-XTARGET2)^2+(yrobot2-YTARGET2)^2);
%    
%    alpha2 = atan(2*U2)/pi;
%    vrobot2 = 50*(1-alpha2);
% 
%    if(distance2 < 3.5*Rrobot)
%     vrobot2 = 0;
%    end
   %%------------- END OF YOUR CODE -------------  
   
   %-------------------------------------------------------------------------------
 

end

sim.terminate();
    
    