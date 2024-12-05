close all;

% Caricamento dati
%load('d1.mat', 'Ob1', 'X', 'X_dot', 'Y', 'Y_dot', 'Z', 'Z_dot', 'xd', 'yd', 'zd', 'd', 'r', 'H');

% Preallocazione per posizioni ostacoli
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


%% Impostazioni grafiche
figure;
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0, 1, 1]);

% Limiti del grafico
xmin = -20; xmax = 30;
ymin = -15; ymax = 20;
zmin = 0; zmax = 90;

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

robot_real = surf(robot_real_x, robot_real_y, robot_real_z, ...
    'FaceColor', [0.9290 0.6940 0.1250], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
robot_ref = surf(robot_ref_x, robot_ref_y, robot_ref_z, ...
    'FaceColor', [0 0.5 1], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
obs1 = surf(obs1_x, obs1_y, obs1_z, ...
    'FaceColor', 'r', 'EdgeColor', 'none', 'FaceAlpha', 0.9);

% Trail (tracce) per posizione reale e di riferimento
trail_real = line('XData', [], 'YData', [], 'ZData', [], 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 3);
trail_ref = line('XData', [], 'YData', [], 'ZData', [], 'Color', [0 0.5 1], 'LineWidth', 3);
trail_obs = line('XData', [], 'YData', [], 'ZData', [], 'Color', [0.9 0 0], 'LineWidth', 3,'LineStyle',':');

% Freccia della velocità
v_vector = quiver3(X(1), Y(1), Z(1), X_dot(1), Y_dot(1), Z_dot(1), ...
                   'Color', 'k', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);

% Testo della velocità
vel = sqrt(X_dot.^2 + Y_dot.^2 + Z_dot.^2);
v_text = text(X(1), Y(1), Z(1) + 1, num2str(vel(1)), 'FontSize', 14, 'Color', 'k');

hold on;
plot3(xd,yd,zd, 'c--', 'LineWidth', 1.5)

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
    
    % Aggiornamento tracce
    x_trail_real = [get(trail_real, 'XData'), xd(k)];
    y_trail_real = [get(trail_real, 'YData'), yd(k)];
    z_trail_real = [get(trail_real, 'ZData'), zd(k)];
    set(trail_real, 'XData', x_trail_real, 'YData', y_trail_real, 'ZData', z_trail_real);
    
    x_trail_ref = [get(trail_ref, 'XData'), X(k)];
    y_trail_ref = [get(trail_ref, 'YData'), Y(k)];
    z_trail_ref = [get(trail_ref, 'ZData'), Z(k)];
    set(trail_ref, 'XData', x_trail_ref, 'YData', y_trail_ref, 'ZData', z_trail_ref);

    x_trail_obs = [get(trail_obs, 'XData'), Pobs1x(k)];
    y_trail_obs = [get(trail_obs, 'YData'), Pobs1y(k)];
    z_trail_obs = [get(trail_obs, 'ZData'), Pobs1z(k)];
    set(trail_obs, 'XData', x_trail_obs, 'YData', y_trail_obs, 'ZData', z_trail_obs);
    
    % Aggiornamento velocità
    set(v_vector, 'XData', X(k), 'YData', Y(k), 'ZData', Z(k), ...
                  'UData', X_dot(k), 'VData', Y_dot(k), 'WData', Z_dot(k));
    
    % Aggiornamento testo della velocità
    set(v_text, 'Position', [X(k), Y(k), Z(k) + 1], 'String', num2str(vel(k)));
    
    xlim([xmin, xmax]);
    ylim([ymin, ymax]);
    zlim([zmin, zmax]);

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
