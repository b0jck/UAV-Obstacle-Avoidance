close all;


%% Impostazioni grafiche
figure;
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0, 1, 1]);

% Limiti del grafico
xmin = -55; xmax = 60;
ymin = -20; ymax = 20;
zmin = 35; zmax = 90;

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
[obs3_x, obs3_y, obs3_z] = generateSphere([Pobs3x(1), Pobs3y(1), Pobs3z(1)], d);
[obs4_x, obs4_y, obs4_z] = generateSphere([Pobs4x(1), Pobs4y(1), Pobs4z(1)], d);
[obs5_x, obs5_y, obs5_z] = generateSphere([Pobs5x(1), Pobs5y(1), Pobs5z(1)], d);
[obs6_x, obs6_y, obs6_z] = generateSphere([Pobs6x(1), Pobs6y(1), Pobs6z(1)], d);
[obs7_x, obs7_y, obs7_z] = generateSphere([Pobs7x(1), Pobs7y(1), Pobs7z(1)], d);

robot_real = surf(robot_real_x, robot_real_y, robot_real_z, ...
    'FaceColor', [0.9290 0.6940 0.1250], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
robot_ref = surf(robot_ref_x, robot_ref_y, robot_ref_z, ...
    'FaceColor', [0 0.5 1], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
obs1 = surf(obs1_x, obs1_y, obs1_z, ...
    'FaceColor', 'r', 'EdgeColor', 'none', 'FaceAlpha', 0.9);
obs2= surf(obs2_x, obs2_y, obs2_z, ...
    'FaceColor', 'y', 'EdgeColor', 'none', 'FaceAlpha', 0.9);
obs3= surf(obs3_x, obs3_y, obs3_z, ...
    'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.9);
obs4= surf(obs4_x, obs4_y, obs4_z, ...
    'FaceColor', 'g', 'EdgeColor', 'none', 'FaceAlpha', 0.9);
obs5= surf(obs5_x, obs5_y, obs5_z, ...
    'FaceColor', 'g', 'EdgeColor', 'none', 'FaceAlpha', 0.9);
obs6= surf(obs6_x, obs6_y, obs6_z, ...
    'FaceColor', 'c', 'EdgeColor', 'none', 'FaceAlpha', 0.9);
obs7= surf(obs7_x, obs7_y, obs7_z, ...
    'FaceColor', 'c', 'EdgeColor', 'none', 'FaceAlpha', 0.9);

% Trail (tracce) per posizione reale e di riferimento
trail_real = line('XData', [], 'YData', [], 'ZData', [], 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 3);
trail_ref = line('XData', [], 'YData', [], 'ZData', [], 'Color', [0 0.5 1], 'LineWidth', 3);
trail_obs1 = line('XData', [], 'YData', [], 'ZData', [], 'Color', [0.9 0 0], 'LineWidth', 3,'LineStyle',':');
trail_obs2 = line('XData', [], 'YData', [], 'ZData', [], 'Color', [1 1 0], 'LineWidth', 3,'LineStyle',':');
trail_obs3 = line('XData', [], 'YData', [], 'ZData', [], 'Color', [0 0 1], 'LineWidth', 3,'LineStyle',':');



% Freccia della velocità
v_vector = quiver3(X(1), Y(1), Z(1), X_dot(1), Y_dot(1), Z_dot(1), ...
                   'Color', 'k', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);

% Testo della velocità
vel = sqrt(X_dot.^2 + Y_dot.^2 + Z_dot.^2);
v_text = text(X(1), Y(1), Z(1) + 1, num2str(vel(1)), 'FontSize', 14, 'Color', 'k');

hold on;
plot3(xd,yd,zd,'c--', 'LineWidth', 2)

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

    [obs3_x, obs3_y, obs3_z] = generateSphere([Pobs3x(k), Pobs3y(k), Pobs3z(k)], d);
    set(obs3, 'XData', obs3_x, 'YData', obs3_y, 'ZData', obs3_z);

    [obs4_x, obs4_y, obs4_z] = generateSphere([Pobs4x(k), Pobs4y(k), Pobs4z(k)], d);
    set(obs4, 'XData', obs4_x, 'YData', obs4_y, 'ZData', obs4_z);

    [obs5_x, obs5_y, obs5_z] = generateSphere([Pobs5x(k), Pobs5y(k), Pobs5z(k)], d);
    set(obs5, 'XData', obs5_x, 'YData', obs5_y, 'ZData', obs5_z);

    [obs6_x, obs6_y, obs6_z] = generateSphere([Pobs6x(k), Pobs6y(k), Pobs6z(k)], d);
    set(obs5, 'XData', obs5_x, 'YData', obs5_y, 'ZData', obs5_z);

    [obs7_x, obs7_y, obs7_z] = generateSphere([Pobs7x(k), Pobs7y(k), Pobs7z(k)], d);
    set(obs7, 'XData', obs7_x, 'YData', obs7_y, 'ZData', obs7_z);
    
    % Aggiornamento tracce
    x_trail_real = [get(trail_real, 'XData'), xd(k)];
    y_trail_real = [get(trail_real, 'YData'), yd(k)];
    z_trail_real = [get(trail_real, 'ZData'), zd(k)];
    set(trail_real, 'XData', x_trail_real, 'YData', y_trail_real, 'ZData', z_trail_real);
    
    x_trail_ref = [get(trail_ref, 'XData'), X(k)];
    y_trail_ref = [get(trail_ref, 'YData'), Y(k)];
    z_trail_ref = [get(trail_ref, 'ZData'), Z(k)];
    set(trail_ref, 'XData', x_trail_ref, 'YData', y_trail_ref, 'ZData', z_trail_ref);

    x_trail_obs1 = [get(trail_obs1, 'XData'), Pobs1x(k)];
    y_trail_obs1 = [get(trail_obs1, 'YData'), Pobs1y(k)];
    z_trail_obs1 = [get(trail_obs1, 'ZData'), Pobs1z(k)];
    set(trail_obs1, 'XData', x_trail_obs1, 'YData', y_trail_obs1, 'ZData', z_trail_obs1);

    x_trail_obs2 = [get(trail_obs2, 'XData'), Pobs2x(k)];
    y_trail_obs2 = [get(trail_obs2, 'YData'), Pobs2y(k)];
    z_trail_obs2 = [get(trail_obs2, 'ZData'), Pobs2z(k)];
    set(trail_obs2, 'XData', x_trail_obs2, 'YData', y_trail_obs2, 'ZData', z_trail_obs2);

    x_trail_obs3 = [get(trail_obs3, 'XData'), Pobs3x(k)];
    y_trail_obs3 = [get(trail_obs3, 'YData'), Pobs3y(k)];
    z_trail_obs3 = [get(trail_obs3, 'ZData'), Pobs3z(k)];
    set(trail_obs3, 'XData', x_trail_obs3, 'YData', y_trail_obs3, 'ZData', z_trail_obs3);
    
    % Aggiornamento velocità
    set(v_vector, 'XData', X(k), 'YData', Y(k), 'ZData', Z(k), ...
                  'UData', X_dot(k), 'VData', Y_dot(k), 'WData', Z_dot(k));
    
    % Aggiornamento testo della velocità
    cbf = H(k);
    clr = [0 0 0];
    if cbf <= 0
        clr = [1 0 0];
    end
    set(v_text, 'Position', [X(k), Y(k), Z(k) + 1], 'String', num2str(cbf), 'Color', clr);

    xlim([xmin, xmax]);
    ylim([ymin, ymax]);
    zlim([zmin, zmax]);

    
    % Genera il frame
    drawnow;

    t(k)
end

%% Funzione per disegnare una sfera
function [x, y, z] = generateSphere(center, radius)
    [sx, sy, sz] = sphere(20);
    x = radius * sx + center(1);
    y = radius * sy + center(2);
    z = radius * sz + center(3);
end
