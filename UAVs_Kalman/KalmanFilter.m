% Parametri
dt = 0.1;  % intervallo di tempo (secondi)
Q = diag([0.1, 0.1, 0.1, 0.01, 0.01, 0.01]);  % covarianza del processo
R = diag([0.5, 0.5, 0.5]);  % covarianza della misura

% Matrice di transizione di stato A
A = [1, 0, 0, dt, 0, 0;
     0, 1, 0, 0, dt, 0;
     0, 0, 1, 0, 0, dt;
     0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 1];

% Matrice di osservazione H
H = [1, 0, 0, 0, 0, 0;
     0, 1, 0, 0, 0, 0;
     0, 0, 1, 0, 0, 0];

% Inizializzazione
x = [0; 3; 20; 0; 0; 0];  % stato iniziale (posizione e velocità)
P = eye(6);       % covarianza iniziale

% Simulazione delle misure e stima
num_steps = 20;  % numero di passi temporali
positions = zeros(num_steps, 3);  % matrice per salvare le posizioni stimate
velocities = zeros(num_steps, 3);  % matrice per salvare le velocità stimate

for k = 1:num_steps
    % Predizione
    x_pred = A * x;  % stima predetta dello stato
    P_pred = A * P * A' + Q;  % predizione della covarianza

    % Simulazione della misurazione (posizione con rumore)
    z = [2*k; 3; 20] + randn(3, 1) * 0.05;  % posizione simulata con rumore

    % Correzione
    y = z - H * x_pred;  % innovazione (errore di misura)
    S = H * P_pred * H' + R;  % matrice di innovazione
    K = P_pred * H' / S;  % guadagno di Kalman

    x = x_pred + K * y;  % aggiornamento dello stato
    P = (eye(6) - K * H) * P_pred;  % aggiornamento della covarianza

    % Salvataggio delle posizioni e velocità stimate
    positions(k, :) = x(1:3)';  % salva la posizione stimata (componente 1, 2, 3)
    velocities(k, :) = x(4:6)';  % salva la velocità stimata (componente 4, 5, 6)
end

% Plot della posizione stimata in 3D
figure;
plot3(positions(:, 1), positions(:, 2), positions(:, 3), 'b-', 'LineWidth', 2);
hold on;
scatter3(positions(:, 1), positions(:, 2), positions(:, 3), 20, 'r', 'filled');
xlabel('Posizione X');
ylabel('Posizione Y');
zlabel('Posizione Z');
title('Traiettoria stimata dell''ostacolo');
grid on;
legend('Posizione stimata', 'Posizioni misurate');
hold off;

% Plot della velocità stimata
figure;
subplot(3, 1, 1);
plot(1:num_steps, velocities(:, 1), 'r-', 'LineWidth', 2);
title('Velocità stimata lungo l''asse X');
xlabel('Tempo');
ylabel('Velocità (m/s)');
grid on;

subplot(3, 1, 2);
plot(1:num_steps, velocities(:, 2), 'g-', 'LineWidth', 2);
title('Velocità stimata lungo l''asse Y');
xlabel('Tempo');
ylabel('Velocità (m/s)');
grid on;

subplot(3, 1, 3);
plot(1:num_steps, velocities(:, 3), 'b-', 'LineWidth', 2);
title('Velocità stimata lungo l''asse Z');
xlabel('Tempo');
ylabel('Velocità (m/s)');
grid on;
