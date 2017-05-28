clc; clear all; 
close all;


%% Nonlinear Control of Inverted Pendulum - Suvadeep Banerjee
% 03/19/2016
% This program implements the control of the classical nonlinear control
% problem, using a linear feedback controller for the stabilization of
% the pendulum around the equilibrium upright position. The initial angle
% is limited to around 30 degrees around the equilibrium upright position
% and the linear feedback controller balances the pendulum from that
% position

%% Parameters of an Inverted Pendulum System

M = 2;      % Mass of the cart in kg
m = 0.2;    % Mass of the pendulum bob in kg
l = 0.31;   % Length of the pendulum in m
g = 9.81;   % Acceleration due to gravity in m/s^2
cart_len = 0.2;    % Cart_length (only for plotting purposes)

%% Simulation Parameters

ts = 1.25e-3;       % Discretization Sampling Time for Continuous Domain Problem
duration = 3;       % Total Time of running

%% Initialization of Parameters

t=0:ts:duration;    % Time Vector
timepts = size(t,2);    % Number of timepoints
x = zeros(4,timepts);   % Initialization for State Vectors
theta_0 = 30*pi/180;      % Initial Angle of Pendulum to start with 
x(1,1) = theta_0;       % State Vector = [Angle Angular_Velocity Cart_Position Cart_Velocity];

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

%% Plant Simulation

for i=2:timepts
    
    theta = x(1,i-1);           % Angular Position
    thetadot = x(2,i-1);        % Angular Velocity
    disp = x(3,i-1);            % Cart Position
    vel = x(4,i-1);             % Cart Velocity

    u = -K*x(:,i-1);

    f1x = (u*cos(theta)-(M+m)*g*sin(theta)+m*l*cos(theta)*sin(theta)*thetadot^2)/(m*l*(cos(theta))^2-(M+m)*l);
    f2x = (u+m*l*sin(theta)*thetadot^2-m*g*cos(theta)*sin(theta))/(M+m-m*(cos(theta))^2);
    
    delta_x = [thetadot f1x vel f2x]';
    x(:,i) = x(:,i-1) + delta_x*ts;  
    
end

final_state = x;
%% Graphical Plots

%% State Plots

set(0,'DefaultAxesFontSize',30);
figure(1)
subplot(221)
% plot(t,180-(final_state(1,:)*180/pi),'linewidth',2);
plot(t,final_state(1,:)*180/pi,'linewidth',2);
title('Angle (degrees)')
% axis([0 20 -100 400])
subplot(222)
plot(t,final_state(2,:),'linewidth',2);
title('Angular Velocity (rad/s)')
subplot(223)
plot(t,final_state(3,:),'r:','linewidth',2)
hold on;
plot(t,final_state(3,:),'linewidth',2);
hold off;
title('Cart Position (m)')
subplot(224)
plot(t,final_state(4,:),'linewidth',2);
title('Cart Velocity (m/s)')

%% Checksum Plot
% 
% figure(2)
% plot(t(2:end),checksum(2:end),'linewidth',2);


%% Animated Plot1
    
% int = 5;    % Plotting Intervel to maintain continuity
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








