function [final_state,control_input] = inv_pend(sim_param,plant_param,initial_state,lin_con_param,nonlin_con_param,switch_angle,sensor_gain,act_fault)

%% Function simulating the inverted pendulum system
% Suvadeep Banerjee 08/28/2016


%% Plant Parameters

M = plant_param(1);
m = plant_param(2);
l = plant_param(3);
g = plant_param(4);

%% Simulation Parameters

ts = sim_param(1);
duration = sim_param(2);

t=0:ts:duration;    % Time Vector
timepts = length(t);    % Number of timepoints


%% Nonlinear Controller Parameters

wn = nonlin_con_param(1);
a0 = nonlin_con_param(2);
b0 = nonlin_con_param(3);
gw = nonlin_con_param(4);
phi = nonlin_con_param(5);
f1 = nonlin_con_param(6);
f2 = nonlin_con_param(7);

Vd = 2*m*g*l;

%% Actuator Initiation

init_state = zeros(2,1);
thy_rat = 3.6;              % Thyristor Bridge Ratio for interface between electronic and electrical power

%% Initial Configuration

x = zeros(4,timepts);
x(1,1) = initial_state;
xd = zeros(timepts,1);  % Reference Trajectory for the nonlinear energy based controller
control_input = zeros(timepts,1);

if abs(x(1,1)) <= switch_angle*pi/180

    nonlinear = 0;          % Nonlinear Controller Switching Parameter
    linear = 1;             % Linear Controller Switching Parameter
else
    
    nonlinear = 1;
    linear = 0;
end

%% System Simulation


% Start of time loop
for i=2:timepts
    
    % Sensor reading
    
    sensed_var = sensor_gain*x(:,i-1);
    
    theta = sensed_var(1);           % Angular Position
    thetadot = sensed_var(2);        % Angular Velocity
    disp = sensed_var(3);            % Cart Position
    vel = sensed_var(4);             % Cart Velocity
    
    
    
    
    
%     while theta > pi
%         theta = theta - 2*pi;
%     end
%     
%     while theta < -pi
%         theta = theta+2*pi;
%     end
%     

% Controller Logic 

% *************** This will be running on hardware and amenable ***********
% *************** to soft errors on digital core ***********************
    
    if abs(theta) < switch_angle*pi/180 || linear == 1
        
        con_sig = -lin_con_param*x(:,i-1);
        linear = 1;
        nonlinear = 0;      % Switch to Linear Controller now
        
    elseif nonlinear == 1
        
        psi = -1*atan((thetadot/wn)/theta-pi);
    
        V = 0.5*m*l^2*thetadot^2+m*g*l*(1-cos(theta-pi));
    
        if abs(V-Vd) >= b0
            a = a0*sign(V-Vd);
        elseif abs(V-Vd) < b0
            a = a0*(V-Vd)/b0;
        end
    
        xd(i) = (a/gw)*sin(psi-pi+phi);
    
        con_sig = (M+m*(sin(theta-pi))^2)*(f1*(xd(i)-disp)-f2*vel)+m*g*cos(theta-pi)*sin(theta-pi)+m*l*thetadot^2*sin(theta-pi);
    end
    
    
    % **************** END OF CONTROLLER CODE RUNNING ON SOFTWARE *********
    
    
    % Actuator module
    
    act_state = act_fault*servo_motor(init_state,con_sig*thy_rat,ts);
           
    u = act_state(1,end)/thy_rat;
    init_state = act_state(:,end);
    control_input(i) = u;
    
    % Actual plant simulation that mimics the physical system
    
    f1x = (u*cos(theta)-(M+m)*g*sin(theta)+m*l*cos(theta)*sin(theta)*thetadot^2)/(m*l*(cos(theta))^2-(M+m)*l);
    f2x = (u+m*l*sin(theta)*thetadot^2-m*g*cos(theta)*sin(theta))/(M+m-m*(cos(theta))^2);
    
    delta_x = [thetadot f1x vel f2x]';
    x(:,i) = x(:,i-1) + delta_x*ts;  
    
%     if i == round(timepts/2)
%         x(1,i) = 30*pi/180;
%     end
    
end

final_state = x;


        
        
        