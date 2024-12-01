clear
close all 
clc

% Use global Parameters to simplify simulation
global A B K Kp Kd mu d d1 f v Ox Oy a b c Obs alpha 

global Pd Xd Yd Zd Pd_dot Xd_dot Yd_dot Zd_dot Pd_dd s r Ox Oy Oz Ux Uy Uz Ob1 Ob2 Ob3 Ob4 Ob5 Ob6 Ob7 H
syms Pd Xd Yd Zd Pd_dot Xd_dot Yd_dot Zd_dot Pd_dd s

%% Trajectory Parameters (for feedforward)
% Origin
Ox = 10;
Oy = 3;
Oz = 60;

% Angular velocity (in Rad/s)
f = pi/4;

%% Feedforward (eight trajectory)

% Trajectory Reference
a = 30;
b = 20;
c = 16;
Pd = [Ox + a*sin(f*s); Oy + b*sin(f*s)*cos(f*s); Oz + c*sin(0.5*f*s)];

% First derivative (velocity reference)
Pd_dot  = [a * f * cos(f * s); b * f * cos(2 * f * s); c * 0.5 * f * cos(0.5 * f * s)];
Xd_dot = Pd_dot(1,:);
Yd_dot = Pd_dot(2,:);
Zd_dot = Pd_dot(3,:);

% Second derivative (acceleration reference for feedforward term)
Pd_dd = [-a * f^2 * sin(f * s); -b * f^2 * sin(2 * f * s); -c * 0.25 * f^2 * sin(0.5 * f * s)];


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
d = 1;
d1 = 3;
r = 5;

% Controller saturation
sat = 20;

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
x0 = [10 3 60 0 0 0];

% Simulation time vector
tspan=0:0.05:30;

% Start simulation
[t, x]=ode45(@simulation,tspan,x0);

% Convert data format for plotting
[~,Xd,Yd,Zd, Ux, Uy, Uz, Ob1, Ob2, Ob3, Ob4, Ob5, Ob6, Ob7, h] = cellfun(@(t,x) simulation(t,x.'), num2cell(t), num2cell(x,2),'uni',0);
H = cell2mat(h);

%Extracting positions of Obstacle
Pobs1x = zeros(1, length(Ob1));
Pobs1y = zeros(1, length(Ob1));
Pobs1z = zeros(1, length(Ob1));

Pobs2x = zeros(1, length(Ob2));
Pobs2y = zeros(1, length(Ob2));
Pobs2z = zeros(1, length(Ob2));

Pobs3x = zeros(1, length(Ob3));
Pobs3y = zeros(1, length(Ob3));
Pobs3z = zeros(1, length(Ob3));

Pobs4x = zeros(1, length(Ob4));
Pobs4y = zeros(1, length(Ob4));
Pobs4z = zeros(1, length(Ob4));

Pobs5x = zeros(1, length(Ob5));
Pobs5y = zeros(1, length(Ob5));
Pobs5z = zeros(1, length(Ob5));

Pobs6x = zeros(1, length(Ob6));
Pobs6y = zeros(1, length(Ob6));
Pobs6z = zeros(1, length(Ob6));

Pobs7x = zeros(1, length(Ob7));
Pobs7y = zeros(1, length(Ob7));
Pobs7z = zeros(1, length(Ob7));


for i = 1:length(Ob1)
    Px = Ob1{i}(1,:);
    Py = Ob1{i}(2,:);
    Pz = Ob1{i}(3,:);
    Pobs1x(i) = double(Px);
    Pobs1y(i) = double(Py);
    Pobs1z(i) = double(Pz);

    Px = Ob2{i}(1,:);
    Py = Ob2{i}(2,:);
    Pz = Ob2{i}(3,:);
    Pobs2x(i) = double(Px);
    Pobs2y(i) = double(Py);
    Pobs2z(i) = double(Pz);

    Px = Ob3{i}(1,:);
    Py = Ob3{i}(2,:);
    Pz = Ob3{i}(3,:);
    Pobs3x(i) = double(Px);
    Pobs3y(i) = double(Py);
    Pobs3z(i) = double(Pz);

    Px = Ob4{i}(1,:);
    Py = Ob4{i}(2,:);
    Pz = Ob4{i}(3,:);
    Pobs4x(i) = double(Px);
    Pobs4y(i) = double(Py);
    Pobs4z(i) = double(Pz);

    Px = Ob5{i}(1,:);
    Py = Ob5{i}(2,:);
    Pz = Ob5{i}(3,:);
    Pobs5x(i) = double(Px);
    Pobs5y(i) = double(Py);
    Pobs5z(i) = double(Pz);

    Px = Ob6{i}(1,:);
    Py = Ob6{i}(2,:);
    Pz = Ob6{i}(3,:);
    Pobs6x(i) = double(Px);
    Pobs6y(i) = double(Py);
    Pobs6z(i) = double(Pz);

    Px = Ob7{i}(1,:);
    Py = Ob7{i}(2,:);
    Pz = Ob7{i}(3,:);
    Pobs7x(i) = double(Px);
    Pobs7y(i) = double(Py);
    Pobs7z(i) = double(Pz);
end

%% Plotting

% Robot Trajectory
figure(1)
title('Robot Trajectory')
xlabel('x')
ylabel('y')

% X,Y position
X = x(:,1);
Y = x(:,2);
Z = x(:,3);

% X,Y velocities
X_dot = x(:,4);
Y_dot = x(:,5);
Z_dot = x(:,6);

% Plot robot trajectory
plot3(X,Y,Z,'color', 'b', 'LineWidth', 2)
xlim([-15,15]);
ylim([-15,15]);
zlim([-15,15]);

hold on



% Plot Obstacles
for i=1:size(Obs,3)

    cx = Obs(1,i);
    cy = Obs(2,i);
    cz = Obs(3,i);
    
    [x, y, z] = sphere(50);

    Xeff = d*x +cx;
    Yeff = d*y +cy;
    Zeff = d*z +cz;

    surf(Xeff, Yeff, Zeff, 'FaceColor', 'r', 'EdgeColor','none');

    plot3(cx,cy, cz, 'bo', 'LineWidth', 5);
end


% Plot Reference Trajectory
xd = cell2mat(Xd);
yd = cell2mat(Yd);
zd = cell2mat(Zd);

plot3(xd, yd, zd, 'color', 'b', 'LineWidth', 2)

% Control Effort
figure(2)
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
% ylim([min([ux;uy])-2,max([ux;uy])+2])

% Robot Velocity
figure(3)
title('Robot Velocities')
xlabel('t')
ylabel('v(t) [m/s]')
plot(t,X_dot, 'b',t, Y_dot,'r','LineWidth',2)
legend('x velocity','y velocity')
ylim([min([X_dot;Y_dot])-2,max([X_dot;Y_dot])+2])

%% Simulation Function
function [dx,Xd,Yd,Zd, Ux, Uy, Uz, Ob1, Ob2, Ob3, Ob4, Ob5, Ob6, Ob7, h] =simulation(t,x)
global A B Pd Pd_dot Pd_dd s Kp Kd Obs ObsDot ObsDotDot mu alpha r f v fo Ox Oy Oz po poPerp a b c

%Obstacle Angular velocity (Circle Trajectory)
foCirc = 4;

%Obstacle Angular velocity (Back and Forth Trajectory)
fo = 1;

deltaFunc1 = 1;


% FeedForward
yref = [Ox + a*sin(f*t); Oy + b*sin(f*t)*cos(f*t); Oz + c*sin(0.5*f*t)];

yref_dot  = [a*f*cos(f*t); b*f*cos(2*f*t); c*0.5*f*cos(0.5*f*t)];

yref_dd = [-a*f^2*sin(f*t); -b*f^2*sin(2*f*t); -c*0.25*f^2*sin(0.5*f*t)];

yref = double(yref);
yref_dot = double(yref_dot);
yref_dd = double(yref_dd);

%Obstacle 1 (Mobile)
Ob1 = [- 20 + 30*sin(fo*t); 6; 70];
Ob1Dot = [30*fo*cos(fo*t); 0; 0];
Ob1DotDot = [-30*fo^2*sin(fo*t); 0; 0];

%Obstacle 2 (Mobile)
Ob2 = [30 - 30*sin(fo*t); 6; 53];
Ob2Dot = [-30*fo*cos(fo*t); 0; 0];
Ob2DotDot = [30*fo^2*sin(fo*t); 0; 0];

%Obstacle 3 (Mobile)
Ob3 = [5*sin(foCirc*t); 6*cos(foCirc*t); 60];
Ob3Dot = [5*foCirc*cos(foCirc*t); -6*foCirc*sin(foCirc*t); 0];
Ob3DotDot = [-5*foCirc^2*sin(foCirc*t); -6*foCirc^2*cos(foCirc*t); 0];

%Obstacle 4 (Fixed)
Ob4 = [48; 3; 70];
Ob4Dot = [0; 0; 0];
Ob4DotDot = [0; 0; 0];

%Obstacle 5 (Fixed)
Ob5 = [34; 3; 70];
Ob5Dot = [0; 0; 0];
Ob5DotDot = [0; 0; 0];

%Obstacle 6 (Fixed)
Ob6 = [19; 6; 62];
Ob6Dot = [0; 0; 0];
Ob6DotDot = [0; 0; 0];

%Obstacle 7 (Fixed)
Ob7 = [19; 10; 62];
Ob7Dot = [0; 0; 0];
Ob7DotDot = [0; 0; 0];

Obs = [Ob1 Ob2 Ob3 Ob4 Ob5 Ob6 Ob7];
ObsDot = [Ob1Dot Ob2Dot Ob3Dot Ob4Dot Ob5Dot Ob7Dot Ob7Dot];
ObsDotDot = [Ob1DotDot Ob2DotDot Ob3DotDot Ob4DotDot Ob5DotDot Ob6DotDot Ob7DotDot];

% UAV position and velocity
Pi = x(1:3);
Pi_dot = x(4:6);

% Nominal control
uNominal = yref_dd + Kd*(yref_dot - Pi_dot) + Kp*(yref-Pi);

% Time visualization
t;

% Compute distances from obstacles
len = size(Obs, 2);
dist = zeros(1, len);
for i=1:len
    dist(1,i) = (Obs(:,i) - Pi)'*(Obs(:,i) - Pi);
end

% Find closest Obstacle
[~, index] = min(dist);
Pobs = Obs(:, index);
PobsDot = ObsDot(:, index);
PobsDotDot = ObsDotDot(:, index);
dist(index) = inf;

% Find second closest Obstacle
[~, ind] = min(dist);
Pobs2 = Obs(:, ind);
 
% Distance from closest Obstacle
V = Pi - Pobs;

% Distance between the two closest obstacles
l = sqrt((Pobs2 - Pobs)'*(Pobs2 - Pobs));

% If robot can't fit between the two closest obstacles, consider a single obstacle that "covers" the two
if sqrt(V'*V) < l+deltaFunc1 + 2*r && l <= 2*(r + deltaFunc1)
    Pobs = (Pobs2 + Pobs)/2;
    deltaFunc1 = 2*(l/2+deltaFunc1+r);
    V = Pi - Pobs;
end

% Print if there is a collision with a obstacle
if(norm(V)<=r+deltaFunc1)
    t
    disp("ALERT")
    % disp((V))
end


V_dot = (Pi_dot - PobsDot);

%Definition of Projection Operators
po = V*((V'*V)^(-1))*V';
poPerp = eye(3) - po;

u_perp = 0;
% Perp component
if rank([V, Pi_dot, PobsDot],  0.1) == 1
    u_perp = ([-V(2); V(1); 0]);
end
    

V_inv = V*((V'*V)^(-1));

%Definition of CBF
h1 = V' * (mu*uNominal + 2*V_dot - mu*PobsDotDot);
h2 = (V'*V + mu*V'*V_dot);
gamma = 12;

%Switching
if h1>0 || h2>deltaFunc1+(r*gamma)
    u = uNominal;
else
    u = ((-2/mu)*(po*V_dot)) + (poPerp*uNominal) + PobsDot + u_perp; %+ 2*V_inv*Pi_dot'*Ob1Dot;
end

% Controller Saturation (if needed)
sat = 100;
if u(1) > sat
    u(1) = sat;
end

if u(1) < -sat
    u(1) = -sat;
end

if u(2) > sat
    u(2) = sat;
end

if u(2) < -sat
    u(2) = -sat;
end

if u(3) > sat
    u(3) = sat;
end

if u(3) < -sat
    u(3) = -sat;
end

% X,Y control
Ux = u(1);
Uy = u(2);
Uz = u(3);

% X,Y reference 
Xd = yref(1);
Yd = yref(2);
Zd = yref(3);

% CBF evaluation index
h = V'*V+mu*V'*V_dot - (deltaFunc1+r);

% Proceed to next simulation step
dx = A*x + B*u;
end