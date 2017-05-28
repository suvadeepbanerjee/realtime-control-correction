clc; clear all; 
close all;


%% Nonlinear Control of Inverted Pendulum - Suvadeep Banerjee
% 11/08/2015
% This program implements the control of the classical nonlinear control
% problem, using a energy-based controller for the nonlinear swing-up of
% the pendulum and a linear feedback controller for the stabilization of
% the pendulum around the equilibrium upright position

%% Parameters of an Inverted Pendulum System

M = 2;      % Mass of the cart in kg
m = 0.2;    % Mass of the pendulum bob in kg
l = 0.31;   % Length of the pendulum in m
g = 9.81;   % Acceleration due to gravity in m/s^2
cart_len = 0.2;    % Cart_length (only for plotting purposes)

plant_param = [M m l g]';       % Plant Parameter Grouping

%% Simulation Parameters

ts = 1.25e-3;       % Discretization Sampling Time for Continuous Domain Problem
duration = 10;       % Total Time of running

sim_param = [ts duration]';

%% Initialization of Parameters

t=0:ts:duration;    % Time Vector
timepts = size(t,2);    % Number of timepoints
x = zeros(4,timepts);   % Initialization for State Vectors
theta_0 = 0*pi/180;      % Initial Angle of Pendulum to start with 
x(1,1) = theta_0;       % State Vector = [Angle Angular_Velocity Cart_Position Cart_Velocity];
initial_state = x;


%% Graphical Plotting Parameters

xline = 0:0.001:l;
yline = 0:0.001:l;

xcart1 = -cart_len:0.01:cart_len;

%% Linear Controller Parameter Determination

% Linearization of the Nonlinear State Space around the unstable
% equilibrium point of theta=0 and thetadot=0

A = [0 1 0 0;g*(M+m)/(l*M) 0 0 0;0 0 0 1;-m*g/M 0 0 0];  % Linearized State Matrix
B = [0 -1/(l*M) 0 1/M]';                                % Linearized Input Matrix

% Design Parameters

pov = 20;               % Percentage of Maximum overshoot
tsettle_2 = 2;          % 2% Settling Time Specification

% Calculation of Damping Factor zeta and Natural Frequency omega_n

k = (log(pov/100))^2;
zeta = sqrt(k/(k+pi^2));
omega_n = 4/(zeta*tsettle_2);

% Calculation of closed loop poles according to design criteria

% Dominant Poles

z1 = exp(-zeta*omega_n*ts+1i*omega_n*ts*sqrt(1-zeta^2));
z2 = exp(-zeta*omega_n*ts-1i*omega_n*ts*sqrt(1-zeta^2));

% Non-dominant poles (Selected using a separation factor of 3

z3 = exp(3*(-zeta*omega_n*ts+1i*omega_n*ts*sqrt(1-zeta^2)));
z4 = exp(3*(-zeta*omega_n*ts-1i*omega_n*ts*sqrt(1-zeta^2)));

% Discretization of the linear state space model

sysc = ss(A,B,eye(4),0);
sysd = c2d(sysc,ts);

% Ackermann's Formula for finding State Feedback Control Law according to
% the desired pole placement

K = place(sysd.a,sysd.b,[z1;z2;z3;z4]);

%% Linear Checksum Error Detection Initialization
checksum = zeros(timepts,1);
cv = 0.38e-2*ones(1,4);
P_mat = cv*A;
Q_mat = cv*B;

%% Nonlinear Energy-based Controller Parameters

wn = sqrt(g/l);
c0 = 0.9;
omega = wn/c0;
zeta_non = 1.2;
H = tf(omega^2,[1,2*zeta_non*omega,omega^2]);

res = evalfr(H,wn*1i);
gw = abs(res);
phi = angle(res);

f1 = omega^2;
f2 = 2*zeta_non*omega;

a0 = 1.3;
b0 = 0.833;

nonlin_con_param = [wn a0 b0 gw phi f1 f2]';      % Nonlinear Controller Parameters
switch_angle = 30;

%% Plant and Controller PCA Simulation
num_trials = 500;
final_state_data = zeros(num_trials,4,timepts);
control_input = zeros(num_trials,timepts);
error_signal = zeros(timepts-1,num_trials);

for i=1:num_trials
    if rand > 0.5
        sensor_fault = 1;
    else sensor_fault = 0;
    end
    
    if rand > 0.5
        actuator_fault = 1;
    else
        actuator_fault = 0;
    end  
    
    if i == num_trials
        [final_state,control_input] = inv_pend(sim_param,plant_param,x,K,nonlin_con_param,switch_angle,0,0);
    else
        [final_state,control_input] = inv_pend(sim_param,plant_param,x,K,nonlin_con_param,switch_angle,sensor_fault,actuator_fault);
    end
    error_signal(:,i) = MARS_eval(final_state,timepts);
    final_state_data(i,:,:) = final_state;
    error_signal(4000,i) = 0;
end



%% Linear Checksum Error Detection

% for i=1:timepts-1;
%     scc = cv*((final_state(:,i+1)-final_state(:,i))/ts);
%     dcc = P_mat*final_state(:,i)+Q_mat*control_input(i);
%     checksum(i) = scc-dcc;
% end

%% Graphical Plots

figure(1)
for i=1:num_trials-1
    plot(t(1:end-1),error_signal(:,i));
    hold on;
end
plot(t(1:end-1),error_signal(:,end),'linewidth',3);
title('Error Signal Over 500 Trials');
xlabel('Time','fontsize',30);
ylabel('Error Signal Magnitude','fontsize',30);


%% State Plots

% set(0,'DefaultAxesFontSize',30);
% figure(1)
% subplot(221)
% % plot(t,180-(final_state(1,:)*180/pi),'linewidth',2);
% plot(t,final_state(1,:)*180/pi,'linewidth',2);
% title('Angle (degrees)')
% % axis([0 20 -100 400])
% xlabel('Time (in s)','fontsize',30);
% subplot(222)
% plot(t,final_state(2,:),'linewidth',2);
% title('Angular Velocity (rad/s)')
% xlabel('Time (in s)','fontsize',30);
% subplot(223)
% plot(t,final_state(3,:)'+normrnd(0,0.04*ones(timepts,1)),'r:','linewidth',2)
% hold on;
% plot(t,final_state(3,:),'linewidth',2);
% hold off;
% title('Cart Position (m)')
% xlabel('Time (in s)','fontsize',30);
% subplot(224)
% plot(t,final_state(4,:),'linewidth',2);
% title('Cart Velocity (m/s)')
% xlabel('Time (in s)','fontsize',30);

%% Checksum Plot
% 
% figure(2)
% plot(t(2:end),checksum(2:end),'linewidth',2);


%% Animated Plot1
    
% int = 20;    % Plotting Intervel to maintain continuity
% 
% for i = 1:floor(timepts/int)
%     figure(2)
%     
%     subplot(211)
%     plot(t(int*i),180-(final_state(1,int*i)*180/pi),'m+','linewidth',3);
%     title('Angle (in degrees)')
%     hold on;
%     xlim([0 duration]);
%     
%     subplot(212)
%     plot(xcart1(round(length(xcart1)/2))+final_state(3,int*i)+xline*sin(final_state(1,int*i)),yline*cos(final_state(1,int*i)),'r','linewidth',3);
%     hold on;
% %     plot(xcart1+final_state(3,int*i),zeros(size(xcart1)),'b','linewidth',3);
%     hold off;
%     axis([-2 0.5 -0.5 0.5]);
% 
% end

%% Animated Plot2

% ang = 180-final_state(1,:)*180/pi;
% x = l*sin(final_state(1,:));
% y = l*cos(final_state(1,:));
% 
% figure('Color','white');
% 
% h2(1) = plot([final_state(3,1),final_state(3,1)+x(1);],[0,y(1)],'r.-','MarkerSize',20,'LineWidth',3);
% hold on;
% h2(2) = plot([final_state(3,1),cart_len+final_state(3,1)],[0,0],'k-','LineWidth',3);
% axis([-1 2 -0.5 0.5]);
% ht = title(sprintf('Time: %0.3f sec',t(1)));
% 
% pos = get(gcf, 'Position');
% width = pos(3);
% height = pos(4);
% 
% handaxes2 = axes('Position',[0.65 0.6 0.2 0.2]);
% plot(t,ang,'LineWidth',2);
% h1 = line(t(1),ang(1),'Marker','.','MarkerSize',20,'Color','m');
% set(handaxes2, 'Box', 'off')
% set(gca,'FontSize',20); 
% 
% % xlabel('Time(sec)');
% % ylabel('Angle(deg)');
% 
% % mov = zeros(height,width,1,timepts,'uint8');
% 
% for id = 1:5:timepts
%     set(h1, 'XData',t(id),'YData',ang(id))
%     set(h2(1), 'XData',[final_state(3,id),final_state(3,id)+x(id)], 'YData', [0,y(id)]);
%     set(h2(2), 'XData',[final_state(3,id),final_state(3,id)+cart_len], 'YData', [0,0]);
%     set(ht, 'String', sprintf('Time: %0.3f sec',t(id)));
%     f = getframe(gcf);
%     
% %     if id == 1
% %         [mov(:,:,1,id),map] = rgb2ind(f.cdata,256, 'nodither');
% %     else
% %         mov(:,:,1,id) = rgb2ind(f.cdata,map,'nodither');
% %     end
% end
% 


