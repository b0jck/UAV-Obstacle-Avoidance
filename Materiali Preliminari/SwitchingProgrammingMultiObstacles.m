clear all
close all
clc

global kp kd mu delta delta1 freq A B K freq Ox Oy R Obstacles numObs a b
global Pd Xd Yd PdDot XdDot YdDot Pd2Dot s
syms Pd Xd Yd PdDot XdDot YdDot Pd2Dot s

%Circle Trajectory and robot start position 
Ox = input('Inserire la coordinata x di partenza del robot:\n');
Oy = input('Inserire la coordinata y di partenza del robot:\n');

%Obstacles Coordinates
numObs = input('Inserire il numero degli ostacoli desiderati:\n');
Obstacles = zeros(2,numObs);
for i=1:numObs
    strInput = sprintf('Ostacolo numero %d. Inserire le coordinate x e y nel formato "[x;y]":\n', i);
    Obstacles(:,i) = input(strInput);
end

%PD Controller
kp = 100; 
kd = 30; 

%CBF - Paper
mu = 0.05;
delta = 2;
delta1 = 5;

%Frequency
freq = pi/2;

%System and Gain Matrix
A = [0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0];
B = [0 0; 0 0; 1 0; 0 1];
K = [kp 0 kd 0; 0 kp 0 kd];

%Desidered Trajectory, Reference
tt = input('Inserire 0 per traiettroia di riferimento circolare e 1 per traiettoria di riferimento ellittica\n');
if tt == 0
    R = input('Inserire il raggio della traiettoria circolare:\n');
    a = R;
    b = R;
    name = 'Safe circular trajectory tracking for mobile robots - SwitchingProgramming';
else
    a = input('Inserire la lunghezza del semiasse maggiore della traiettoria ellittica:\n');
    b = input('Inserire la lunghezza del semiasse minore della traiettoria ellittica:\n');
    name = 'Safe elliptical trajectory tracking for mobile robots - SwitchingProgramming';
end
Pd = [Ox + a*sin(freq*s); Oy + b*cos(freq*s)];
PdDot = [(freq*a*cos(freq*s)); (-freq*b*sin(freq*s))];
Pd2Dot = [(-(power(freq,2))*a*sin(freq*s)); (-(power(freq,2))*b*cos(freq*s))];
Xd = Pd(1,:);
Yd = Pd(1,:);
XdDot = PdDot(1,:);
YdDot = PdDot(1,:);

%Feedback
tspan = 0 : 0.05 : 10;
x0 = [Ox Oy 0 0];
[t x] = ode45(@systemControl, tspan, x0);

posR = x(1:2,:);

%Plot
figure('Name', name)
X = x(:,1);
Y = x(:,2);
plot(X,Y, 'b', 'LineWidth', 1, 'DisplayName', 'Robot Trajectory', 'HandleVisibility','on')
axis equal
hold on
for i=1:numObs
    plot(Obstacles(1,i), Obstacles(2,i), 'o', 'LineWidth', 0.7, 'HandleVisibility','off');
    plot(nsidedpoly(1000, 'Center', Obstacles(:,i)', 'Radius', delta),'FaceColor', 'r', 'HandleVisibility','off');
    plot(nsidedpoly(1000, 'Center', Obstacles(:,i)', 'Radius', delta1), 'FaceColor', 'r', 'HandleVisibility','off');
end

%Define the rotation angle of the ellipse or the circle
angle = 0;    
    
%Generate the ellipse or circle
tel = linspace(0, 2*pi, 100);  
ellipseX = Ox + a*cos(t)*cosd(angle) - b*sin(t)*sind(angle);
ellipseY = Oy + a*cos(t)*sind(angle) + b*sin(t)*cosd(angle);
    
%Plot the ellipse or circle
plot(ellipseX, ellipseY, 'r', 'LineWidth', 0.8, 'LineStyle', '--', 'DisplayName','Reference', 'HandleVisibility','on');
axis equal;  

%Design of figure
str = 'Obstacles position: [';
for i = 1:size(Obstacles, 2)
    str = [str sprintf('(%0.1f,%0.1f)', Obstacles(1,i), Obstacles(2,i))];
    if i < size(Obstacles, 2)
        str = [str ';'];
    end
end
str = [str ']'];

titlePosR = sprintf('Robot Start Position: [%0.1f,%0.1f], ', Ox, Oy);
titleFin = [titlePosR str];
title(titleFin);
xlabel('x')
ylabel('y')
legend('show', 'Location', 'best');


%Control
function [xDot, ux, uy] = systemControl(t,x)
    global A B Pd PdDot Pd2Dot s kp kd mu delta1 Obstacles numObs a b freq Ox Oy
    
    %Feedforward
    yRef = [Ox + a*sin(freq*t); Oy + b*cos(freq*t)];
    yRefDot = [(freq*a*cos(freq*t)); (-freq*b*sin(freq*t))];
    yRef2Dot = [(-(power(freq,2))*a*sin(freq*t)); (-(power(freq,2))*b*cos(freq*t))];
    yRef = double(yRef);
    yRefDot = double(yRefDot);
    yRef2Dot = double(yRef2Dot);
    
    %Position & Velocity
    posR = x(1:2);
    velR = x(3:4);

    %Nominal control input in Dtrack
    uNominal = yRef2Dot + kd*(yRefDot - velR) + kp*(yRef - posR);
    d = zeros(1, numObs);
    for i=1:numObs
        d(1,i) = (Obstacles(:,i) - posR)'*(Obstacles(:,i) - posR);
    end
    index = find(d == min(d));
    closest = Obstacles(:, index);
    t

    %Definition of Projection Operators
    po = ((posR-closest)*((((posR-closest)')*(posR-closest))^(-1))*(posR-closest)');
    poPerp = eye(2) - po;

    %Definition of CBF
    h1 = ((posR- closest)' * (mu*uNominal + 2*velR));
    h2 = ((posR-closest)' * (posR-closest)) + mu*((posR-closest)' * velR);

    %Switching
    if h1>0 || h2>delta1
        u = uNominal;
    else
        u = ((-2/mu)*(po*velR)) + (poPerp*uNominal);
    end

    ux = u(1);
    uy = u(2);
    xDot = A*x + B*u;

end

        
    



    


