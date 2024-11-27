clear all
close all
clc

global kp kd mu delta delta1 freq A B K freq Ox Oy R Obstacles numObs RR a b 
global Pd Xd Yd PdDot XdDot YdDot Pd2Dot s
syms Pd Xd Yd PdDot XdDot YdDot Pd2Dot s

%Circle Trajectory and robot start position 
Ox = input('Inserire la coordinata x di partenza del robot:\n');
Oy = input('Inserire la coordinata y di partenza del robot:\n');

freq = pi/4;
RR = input('Inserire il raggio del robot:\n');

%Obstacles Coordinates
delta = 2;
delta1 = 5;
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
tspan = 0 : 0.05 : 15;
x0 = [Ox Oy 0 0];
[t x] = ode45(@systemControl, tspan, x0);
[~,xD,~] = cellfun(@(t,x) systemControl(t,x.'), num2cell(t), num2cell(x,2),'uni',0);
[~,~,yD] = cellfun(@(t,x) systemControl(t,x.'), num2cell(t), num2cell(x,2),'uni',0);

posR = x(1:2,:);

%Plot
figure('Name', name)

%Trajectory of the robot and related surface lines
X = x(:,1);
Y = x(:,2);
XDot = x(:,3);
YDot = x(:,4);
dim = length(X);
x1 = zeros(dim,1);
y1 = zeros(dim,1);
x2 = zeros(dim,1);
y2 = zeros(dim,1);
for i=1:dim
    d = sqrt(XDot(i)^2+YDot(i)^2);
    sin_a = YDot(i)/d;
    cos_a = XDot(i)/d;
    
    x1(i) = X(i)-RR*sin_a;
    y1(i) = Y(i)+RR*cos_a;

    x2(i) = X(i)+RR*sin_a;
    y2(i) = Y(i)-RR*cos_a;
end

X1 = x1;
Y1 = y1;
X2 = x2;
Y2 = y2;
plot(X,Y, 'b', 'LineWidth', 1, 'DisplayName', 'Robot Trajectory', 'HandleVisibility','off')
hold on
plot(X1, Y1, 'Color', '#FFD480', 'LineWidth', 1, 'LineStyle','--', 'DisplayName', 'Robot Top Edge', 'HandleVisibility','off')
plot(X2, Y2, 'g', 'LineWidth', 1, 'LineStyle','--', 'DisplayName', 'Robot Bottom Edge', 'HandleVisibility','off')
axis equal
hold on

% Fill area in between
for i=2:length(X1)
    plot([X1(i) X2(i)], [Y1(i) Y2(i)], 'LineWidth', 5, 'Color', 'y', 'HandleVisibility','off')
end

plot(X,Y, 'b', 'LineWidth', 1, 'DisplayName', 'Robot Trajectory')
hold on
plot(X1, Y1, 'Color', '#FFD480', 'LineWidth', 1, 'LineStyle','--', 'DisplayName', 'Robot Top Edge')
plot(X2, Y2, 'g', 'LineWidth', 1, 'LineStyle','--', 'DisplayName', 'Robot Bottom Edge')
axis equal

%Obstacles
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

titlePosR = sprintf('Robot Radius: %0.1f. Robot Start Position: [%0.1f,%0.1f], ', RR,Ox,Oy);
titleFin = [titlePosR str];
title(titleFin);
xlabel('x')
ylabel('y')
legend('show', 'Location', 'best')

xD = cell2mat(xD);
yD = cell2mat(yD);

%Control
function [xDot, xD, yD] = systemControl(t,x)
    global A B Pd PdDot Pd2Dot s kp kd mu delta1 Obstacles numObs delta freq a b Ox Oy RR

    deltaFunc = delta;
    deltaFunc1 = delta1;
    
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

    %Find two closest Obstacles
    vettDist = zeros(1, numObs);
    for i=1:numObs
        vettDist(1,i) = (Obstacles(:,i) - posR)'*(Obstacles(:,i) - posR);
    end
    index1 = find(vettDist == min(vettDist));
    closest1 = Obstacles(:, index1);

    vettDist(index1) = inf;

    index2 = find(vettDist == min(vettDist));
    closest2 = Obstacles(:, index2);
    vettDist(index2) = inf;

    %Check the robot passes between the two obstacles
    beta = 8;
    gamma = 24;
    distResidua = posR - closest1;
    dObs1Obs2 = sqrt((closest2 - closest1)'*(closest2 - closest1));
    ghost = closest1;

    if (sqrt(distResidua'*distResidua) < dObs1Obs2+deltaFunc) && (dObs1Obs2 <= 2*(RR+deltaFunc))
        ghost = (closest2 + closest1)/2;
        deltaFunc1 = beta*(dObs1Obs2/2+deltaFunc1+RR);
    end
    t

    %Definition of Projection Operators
    po = ((posR-ghost)*((((posR-ghost)')*(posR-ghost))^(-1))*(posR-ghost)');
    poPerp = eye(2) - po;

    %Definition of CBF
    h1 = ((posR- ghost)' * (mu*uNominal + 2*velR));
    h2 = ((posR-ghost)' * (posR-ghost)) + mu*((posR-ghost)' * velR);

    %Switching
    if h1>0 || h2>deltaFunc1+(gamma*RR)
        u = uNominal;
    else
        u = ((-2/mu)*(po*velR)) + (poPerp*uNominal);
    end

    xD = yRef(1);
    yD = yRef(2);
    xDot = A*x + B*u;

end

        
    



    


