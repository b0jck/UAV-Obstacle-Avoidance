% Esempio di dati
x = 0:0.1:10;
y = sin(x);

% Variabile del tempo
t = 5; % esempio di valore di tempo

% Grafico
plot(x, y, 'b-', 'LineWidth', 2);
hold on;

% Titolo e assi
title('Esempio di grafico con legenda');
xlabel('x');
ylabel('y');

% Aggiungere una leggenda con il valore di t
legend(sprintf('Seno (t = %.2f)', t), 'Location', 'best');

hold off;
