close all;

% Preallocazione per posizioni ostacoli
Pobs1x = zeros(1, length(Ob1));
Pobs1y = zeros(1, length(Ob1));
Pobs1z = zeros(1, length(Ob1));

Pobs2x = zeros(1, length(Ob2));
Pobs2y = zeros(1, length(Ob2));
Pobs2z = zeros(1, length(Ob2));

for i = 1:length(Ob1)
    Px1 = Ob1{i}(1,:);
    Py1 = Ob1{i}(2,:);
    Pz1 = Ob1{i}(3,:);

    Px2 = Ob2{i}(1,:);
    Py2 = Ob2{i}(2,:);
    Pz2 = Ob2{i}(3,:);

    Pobs1x(i) = double(Px1);
    Pobs1y(i) = double(Py1);
    Pobs1z(i) = double(Pz1);
    
    Pobs2x(i) = double(Px2);
    Pobs2y(i) = double(Py2);
    Pobs2z(i) = double(Pz2);
end


%% Impostazioni grafiche
figure;
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0, 1, 1]);

% Limiti del grafico
xmin = -20; xmax = 20;
ymin = -20; ymax = 20;
zmin = 0; zmax = 20;

xlim([xmin, xmax]);
ylim([ymin, ymax]);
zlim([zmin, zmax]);

hold on;
grid on;

% Imposta la vista 3D
view(3);
axis equal;

% Genera sfere iniziali per robot reale, robot di riferimento e ostacolo
[robot_real_x, robot_real_y, robot_real_z] = generateSphere([xd(1), yd(1), zd(1)], r);
[robot_ref_x, robot_ref_y, robot_ref_z] = generateSphere([X(1), Y(1), Z(1)], r);
[obs1_x, obs1_y, obs1_z] = generateSphere([Pobs1x(1), Pobs1y(1), Pobs1z(1)], d);
[obs2_x, obs2_y, obs2_z] = generateSphere([Pobs2x(1), Pobs2y(1), Pobs2z(1)], d);

robot_real = surf(robot_real_x, robot_real_y, robot_real_z, ...
    'FaceColor', [0.9290 0.6940 0.1250], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
robot_ref = surf(robot_ref_x, robot_ref_y, robot_ref_z, ...
    'FaceColor', [0 0.5 1], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
obs1 = surf(obs1_x, obs1_y, obs1_z, ...
    'FaceColor', 'r', 'EdgeColor', 'none', 'FaceAlpha', 0.9);
obs2= surf(obs2_x, obs2_y, obs2_z, ...
    'FaceColor', 'y', 'EdgeColor', 'none', 'FaceAlpha', 0.9);

% Trail (tracce) per posizione reale e di riferimento
trail_real = plot3([], [], [], '-', 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 2);
trail_ref = plot3([], [], [], '-', 'Color', [0 0.5 1], 'LineWidth', 2);

% Freccia della velocità
v_vector = quiver3(X(1), Y(1), Z(1), X_dot(1), Y_dot(1), Z_dot(1), ...
                   'Color', 'k', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);

% Testo della velocità
vel = sqrt(X_dot.^2 + Y_dot.^2 + Z_dot.^2);
v_text = text(X(1), Y(1), Z(1) + 1, num2str(vel(1)), 'FontSize', 14, 'Color', 'k');

hold on;
plot3(xd,yd,zd,'color', 'b', 'LineWidth', 2)

%% Animazione
for k = 2:length(xd)
    % Aggiornamento posizione robot reale
    [robot_real_x, robot_real_y, robot_real_z] = generateSphere([xd(k), yd(k), zd(k)], r);
    set(robot_real, 'XData', robot_real_x, 'YData', robot_real_y, 'ZData', robot_real_z);
    
    % Aggiornamento posizione robot di riferimento
    [robot_ref_x, robot_ref_y, robot_ref_z] = generateSphere([X(k), Y(k), Z(k)], r);
    set(robot_ref, 'XData', robot_ref_x, 'YData', robot_ref_y, 'ZData', robot_ref_z);
    
    % Aggiornamento posizione ostacoli
    [obs1_x, obs1_y, obs1_z] = generateSphere([Pobs1x(k), Pobs1y(k), Pobs1z(k)], d);
    set(obs1, 'XData', obs1_x, 'YData', obs1_y, 'ZData', obs1_z);

    [obs2_x, obs2_y, obs2_z] = generateSphere([Pobs2x(k), Pobs2y(k), Pobs2z(k)], d);
    set(obs2, 'XData', obs2_x, 'YData', obs2_y, 'ZData', obs2_z);
    
    % Aggiornamento tracce
    x_trail_real = [get(trail_real, 'XData'), xd(k)];
    y_trail_real = [get(trail_real, 'YData'), yd(k)];
    z_trail_real = [get(trail_real, 'ZData'), zd(k)];
    set(trail_real, 'XData', x_trail_real, 'YData', y_trail_real, 'ZData', z_trail_real);
    
    x_trail_ref = [get(trail_ref, 'XData'), X(k)];
    y_trail_ref = [get(trail_ref, 'YData'), Y(k)];
    z_trail_ref = [get(trail_ref, 'ZData'), Z(k)];
    set(trail_ref, 'XData', x_trail_ref, 'YData', y_trail_ref, 'ZData', z_trail_ref);
    
    % Aggiornamento velocità
    set(v_vector, 'XData', X(k), 'YData', Y(k), 'ZData', Z(k), ...
                  'UData', X_dot(k), 'VData', Y_dot(k), 'WData', Z_dot(k));
    
    % Aggiornamento testo della velocità
    set(v_text, 'Position', [X(k), Y(k), Z(k) + 1], 'String', num2str(vel(k)));
    
    % Genera il frame
    drawnow;
end

%% Funzione per disegnare una sfera
function [x, y, z] = generateSphere(center, radius)
    [sx, sy, sz] = sphere(20);
    x = radius * sx + center(1);
    y = radius * sy + center(2);
    z = radius * sz + center(3);
end
