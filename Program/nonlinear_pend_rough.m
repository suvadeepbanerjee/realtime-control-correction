clc; clear all; 
close all;

%% Parameters of an Inverted Pendulum System

M = 2;      % Mass of the cart in kg
m = 0.2;    % Mass of the pendulum bob in kg
l = 0.31;   % Length of the pendulum in m
g = 9.81;   % Acceleration due to gravity in m/s^2
cart_len = 0.05;

%% Simulation Parameters

ts = 1.25e-3;       % Discretization Sampling Time for Continuous Domain Problem
duration = 30;       % Total Time of running

xline = 0:0.001:l;
yline = 0:0.001:l;

xcart1 = -cart_len:0.01:cart_len;


% Initialization of Parameters

t=0:ts:duration;    % Time Vector
timepts = size(t,2);    % Number of timepoints
error = zeros(timepts,1);   % Initialization for Checksum Error Vector
error_m = zeros(4,timepts); % Initialization for Error Vector by comparing with model
x = zeros(4,timepts);   % Initialization for State Vectors
theta_0 = 180*pi/180;      % Initial Angle of Pendulum to start with (range -0.15*pi to 0.15*pi)
x(1,1) = theta_0;

% x(:,1) = [0.5411 0.33 -0.111 0.3956]';
xd = zeros(timepts,1);  % Reference Trajectory for the nonlinear energy based controller
control = zeros(timepts,1);
switch_var = zeros(timepts,1);

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


% Nonlinear Controller Parameters

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

Vd = 2*m*g*l;

a0 = 1.3;
b0 = 0.833;

nonlinear = 1;
linear = 0;

%% Control Loop

for i=2:timepts
    
    theta = x(1,i-1);
    thetadot = x(2,i-1);
    disp = x(3,i-1);
    vel = x(4,i-1);
    
    if abs(theta) < 30*pi/180 || linear == 1
        
        
%          u = -K*x(:,i-1);
         u = -(K(1)*theta+K(2)*thetadot+K(3)*disp+K(4)*vel);
         linear = 1;
         nonlinear = 0;
         
         
    elseif nonlinear == 1 
        
        psi = -1*atan((thetadot/wn)/theta-pi);
    
        V = 0.5*m*l^2*thetadot^2+m*g*l*(1-cos(theta-pi));
    
        if abs(V-Vd) >= b0
            a = a0*sign(V-Vd);
        elseif abs(V-Vd) < b0
            a = a0*(V-Vd)/b0;
        end
    
        xd(i) = (a/gw)*sin(psi-pi+phi);
    
        u = (M+m*(sin(theta-pi))^2)*(f1*(xd(i)-disp)-f2*vel)+m*g*cos(theta-pi)*sin(theta-pi)+m*l*thetadot^2*sin(theta-pi);
    
    end

    
    control(i) = u;
    
    % Compute functions of the original nonlinear state equation
    
    f1x = (u*cos(theta)-(M+m)*g*sin(theta)+m*l*cos(theta)*sin(theta)*thetadot^2)/(m*l*(cos(theta))^2-(M+m)*l);
    f2x = (u+m*l*sin(theta)*thetadot^2-m*g*cos(theta)*sin(theta))/(M+m-m*(cos(theta))^2);

%     f2x = (1/(M+m*(sin(theta-pi))^2))*(-m*g*cos(theta-pi)*sin(theta-pi)-m*l*thetadot^2*sin(theta-pi)+u);
%     f1x = (-g/l)*sin(theta-pi)+(f2x/l)*cos(theta-pi);


    
    % Find out the update in the states and update accordingly
    
    delta_x = [thetadot f1x vel f2x]';
    x(:,i) = x(:,i-1) + delta_x*ts;     
    
    
        
    
end

%% Plots

figure(1)
subplot(221)
plot(t,180-(x(1,:)*180/pi),'linewidth',2);
title('Angle (in degrees)')
subplot(222)
plot(t,x(2,:),'linewidth',2);
title('Angular Velocity')
subplot(223)
plot(t,x(3,:),'linewidth',2);
title('Cart Position')
subplot(224)
plot(t,x(4,:),'linewidth',2);
title('Cart Velocity')

% for i = 1:floor(timepts/6)
% 
% figure(4)
% plot(xcart1(round(length(xcart1)/2))+x(3,6*i)+xline*sin(x(1,6*i)),yline*cos(x(1,6*i)),'r','linewidth',3);
% hold on;
% plot(round(length(xcart1)/2)+x(3,6*i)+xline(end)*sin(x(1,6*i)),yline(end)*cos(x(1,6*i)),'ro','linewidth',3);
% plot(xcart1+x(3,6*i),zeros(size(xcart1)),'b','linewidth',3);
% hold off;
% axis([-2 0.5 -0.5 0.5]);
% 
% end


figure(2)
plot(t,control)

