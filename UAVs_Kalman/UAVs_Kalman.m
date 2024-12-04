clear
close all 
clc

% Use global Parameters to simplify simulation
global A B K Kp Kd mu d d1 f Ox Oy R Obs alpha dt Ak Hk Pk Jk Qk xk i

global Pd Xd Yd Zd Pd_dot Xd_dot Yd_dot Zd_dot Pd_dd s r a b Ox Oy Oz Ux Uy Uz Ob1 Ob2 H Ob1_pred Ob1Dot_pred 
syms Pd Xd Yd Zd Pd_dot Xd_dot Yd_dot Zd_dot Pd_dd s

%% Trajectory Parameters (for feedforward)
% Origin
Ox = 20;
Oy = 3;
Oz = 20;

% Ellipse's axes
a = 20;
b = 10;

% Angular velocity (in Rad/s)
f = pi/4;

%% Feedforward (elliptic trajectory)

% Trajectory Reference
Pd = [  Ox - f*s; Oy; Oz];

% First derivative (velocity reference)
Pd_dot  = [ -f; 0; 0];

Xd_dot = Pd_dot(1,:);
Yd_dot = Pd_dot(2,:);
Zd_dot = Pd_dot(3,:);

% Second derivative (acceleration reference for feedforward term)
Pd_dd = [0; 0; 0];

%% Obstacles Coordinates

% In this scenario, all obstacles are moving. Otherwise, specify constant
% Position of each obstacle

%% Controller's parameters 
% Proportional And derivative Gains
Kp = 100;
Kd = 30;

% Gain Matrix
K = [   Kp  0   0   Kd  0   0;
        0   Kp  0   0   Kd  0
        0   0   Kp  0   0   Kd];

% CBF Parameters
alpha = 5;
mu = 0.05;
d = 2;
d1 = 5;
r = 1;

% Controller saturation
sat = 100;

%% System Dynamics' Matrices

A = [   0 0 0 1 0 0;
        0 0 0 0 1 0;
        0 0 0 0 0 1;
        0 0 0 0 0 0;
        0 0 0 0 0 0; 
        0 0 0 0 0 0];


B = [   0 0 0;
        0 0 0;
        0 0 0;
        1 0 0;
        0 1 0;
        0 0 1];

%% Closed Loop system simulation

% Initial state (robot in the origin, with zero velocity)
x0 = [20 3 20 0 0 0];

% Simulation time vector
tspan=0:0.05:30;

%% Kalman Filter
%Setup Kalman Filter
dt = 0.05;  % Sampling time
Qk = diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1]);  % Process Covariance
Jk = diag([0.05, 0.05, 0.05]);  % Measurements Covariance

% Transition Matrix 
omega_dt = f * dt; % Termine della velocità angolare
Ak = [1, 0, 0, dt, 0, 0;
     0, 1, 0, 0, dt, 0;
     0, 0, 1, 0, 0, dt;
     0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 1];


% Observation matrix
Hk = [1, 0, 0, 0, 0, 0;
     0, 1, 0, 0, 0, 0;
     0, 0, 1, 0, 0, 0];

% Initialization
x0k = [0; 10; 20; 0; 0; 0];  % initial state
Pk = eye(6);       % initial covariance

xk = zeros(6, length(tspan));
xk(:,1) = x0k;

% Parameter for Kalman Filter
i = 1;

% Start simulation
[t, x]=ode45(@simulation,tspan,x0);

% Convert data format for plotting
[~,Xd,Yd,Zd, Ux, Uy, Uz, Ob1, h] = cellfun(@(t,x) simulation(t,x.'), num2cell(t), num2cell(x,2),'uni',0);
H = cell2mat(h)

%Extracting positions of Obstacle
Pobs1x = zeros(1, length(Ob1));
Pobs1y = zeros(1, length(Ob1));
Pobs1z = zeros(1, length(Ob1));

for i = 1:length(Ob1)
    Px = Ob1{i}(1,:);
    Py = Ob1{i}(2,:);
    Pz = Ob1{i}(3,:);
    Pobs1x(i) = double(Px);
    Pobs1y(i) = double(Py);
    Pobs1z(i) = double(Pz);
end

%% Plotting

% X,Y,Z position
X = x(:,1);
Y = x(:,2);
Z = x(:,3);

% X,Y,Z velocities
X_dot = x(:,4);
Y_dot = x(:,5);
Z_dot = x(:,6);

% Plot Reference Trajectory
xd = cell2mat(Xd);
yd = cell2mat(Yd);
zd = cell2mat(Zd);

% Control Effort
figure(1)
title('Control Effort')
xlabel('t')
ylabel('u(t) [m/s^2]')
ux = cell2mat(Ux);
uy = cell2mat(Uy);
uz = cell2mat(Uz);
plot(t,ux, 'b',t, uy,'r', t, uz, 'y', 'LineWidth',2)
yline(sat, 'k--','Label','Saturation')
yline(-sat, 'k--','Label','Saturation')
legend('x acceleration','y acceleration', 'z acceleration')
ylim([min([ux;uy;uz])-2,max([ux;uy;uz])+2])

% UAV Velocity
figure(2)
title('Robot Velocities')
xlabel('t')
ylabel('v(t) [m/s]')
plot(t,X_dot, 'b',t, Y_dot,'r', t, Z_dot,'y', 'LineWidth',2)
legend('x velocity','y velocity', 'z velocity')
ylim([min([X_dot;Y_dot;Z_dot])-2,max([X_dot;Y_dot;Z_dot])+2])


figure(3)
T = [0 tspan];
subplot(2,1,1)
plot(T,xk(4,:) , 'r', T, xk(5,:), 'y', T, xk(6,:), 'b', 'Linewidth', 2)
title('Estimated velocity of the obstacle');
xlabel('t');
ylabel('v(t) [m/s]');
legend('x estimated velocity','y estimated velocity', 'z estimated velocity')
grid on;

subplot(2, 1, 2); 
plot(T, 10*f*cos(f*T), 'r', T, -10*f*sin(f*T), 'y', T, 0*ones(length(T)), 'b', 'Linewidth', 2);
title('Real velocity of the obstacle');
xlabel('t');
ylabel('v(t) [m/s]');
legend('x real velocity','y real velocity', 'z real velocity')
grid on;

%% Simulation Function
function [dx,Xd,Yd,Zd, Ux, Uy, Uz, Ob1, h] =simulation(t,x)
global A B Pd Pd_dot Pd_dd s Kp Kd Obs mu alpha r a b f Ox Oy Oz po poPerp dt Ak Hk Pk Jk Qk xk i Ob1_pred Ob1Dot_pred Ob1Dot

deltaFunc1 = 2;

% FeedForward
yref = [Ox - f*t; Oy; Oz];

yref_dot  = [-f; 0; 0];

yref_dd = [0; 0; 0];

yref = double(yref);
yref_dot = double(yref_dot);
yref_dd = double(yref_dd);

% Kalman Filter for Obstacle estimation
% Prediction

x_pred = Ak * xk(:,i);  
P_pred = Ak * Pk * Ak' + Qk;  

% Measurement simulation
zk = [10*sin(f*t); 10*cos(f*t); 20] + randn(3, 1) * 0.05;  

% Correction
yk = zk - Hk * x_pred;  
Sk = Hk * P_pred * Hk' + Jk;  
Kk = P_pred * Hk' / Sk;  

xk(:,i+1) = x_pred + Kk * yk;  

Pk = (eye(6) - Kk * Hk) * P_pred;  

% Estimated Position and velocity 
xkk=xk(:,i);
Ob1_pred = double(xkk(1:3));  
Ob1Dot_pred = double(xkk(4:6));  

% Real position, velocity and acceleration of the obstacle
Ob1 = [10*sin(f*t); 10*cos(f*t); 20];
Ob1Dot = [10*f*cos(f*t); -10*f*sin(f*t); 0];
Ob1DotDot = [-10*f^2*sin(f*t); -10*f^2*cos(f*t); 0];

Ob1 = double(Ob1);
Ob1Dot = double(Ob1Dot);
Ob1DotDot = double(Ob1DotDot);

Obs = Ob1;

% Position and Velocity
Pi = x(1:3);
Pi_dot = x(4:6);

% Nominal control
uNominal = yref_dd + Kd*(yref_dot - Pi_dot) + Kp*(yref-Pi);

t

V = (Pi - Ob1);
V_dot = (Pi_dot - Ob1Dot_pred);
%Definition of Projection Operators
po = V*((V'*V)^(-1))*V';
poPerp = eye(3) - po;

u_perp = 0;
% Perp component
if rank([V, Pi_dot, x(4:6)],  0.1) == 1
    u_perp = ([-V(2); V(1); 0]);
end
    
%Definition of CBF
h1 = V' * (mu*uNominal + 2*V_dot - mu*Ob1DotDot);
h2 = (V'*V + mu*V'*V_dot);
gamma = 12;

%Switching
if h1>0 || h2>deltaFunc1+(r*gamma)
    u = uNominal;
else
    u = ((-2/mu)*(po*V_dot)) + (poPerp*uNominal) + Ob1DotDot + u_perp; %+ 2*V_inv*Pi_dot'*Ob1Dot;
end


% X,Y reference 
Xd = yref(1);
Yd = yref(2);
Zd = yref(3);

% CBF evaluation index
h = V'*V+mu*V'*V_dot - (deltaFunc1+r);

% % Controller Saturation (if needed)
% sat = 100;
% if u(1) > sat
%     u(1) = sat;
% end
% 
% if u(1) < -sat
%     u(1) = -sat;
% end
% 
% if u(2) > sat
%     u(2) = sat;
% end
% 
% if u(2) < -sat
%     u(2) = -sat;
% end
% 
% if u(3) > sat
%     u(3) = sat;
% end
% 
% if u(3) < -sat
%     u(3) = -sat;
% end

% X,Y,Z control
Ux = u(1);
Uy = u(2);
Uz = u(3);

% Proceed to next simulation step
dx = A*x + B*u;

if t > (i-1)*0.05
    i = i+1;
end

end
