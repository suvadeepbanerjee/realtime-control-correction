function act_state = servo_motor(init_state,theta_ref,ts_orig)

% Implementation of a Servo Motor Actuator for Inverted Pendulum System

%% Parameters

zeta = 0.9;         % Damping Factor
omega_n = 0.5;        % Natural Frequency of Oscillation
kt = 0.057;         % Torque Constant
kb = 0.052;         % Back-emf constant
J1 = 900e-3;        % Moment of Inertia

alpha = -(kt*kb)/J1;
beta = kt/J1;

a1 = 2*zeta*omega_n;
a2 = omega_n^2;         % Coefficients of desired characteristic polynomial

g1 = a2/beta;
g2 = (a1+alpha)/beta;

mat_A = [0 1;-beta*g1 alpha-beta*g2];
mat_B = [0 beta*g1]';


%% Motor Simulation

ts = ts_orig/10;       % Sampling Interval
t = 0:ts:10*ts;
timepts = size(t,2);

x = zeros(2,timepts);       % State of the motor - Angle and Angular velocity
x(:,1) = init_state;

for i=1:timepts-1
    delta_x = mat_A*x(:,i)+mat_B*theta_ref;
    x(:,i+1) = x(:,i)+delta_x;
end

act_state = x;


