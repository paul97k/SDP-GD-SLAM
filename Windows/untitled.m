clc;
clear;

% Parameters
nT_max = 100;      % Maximum nT value
step_size = 1;    % Increment step for nT
pause_time = 0.1; % Pause time between frames

% Create figure
figure;
hold on;
grid on;
xlabel('nT');
ylabel('nT^3');
title('nT^3');

% Initialize plot
h = plot(NaN, NaN, 'b', 'LineWidth', 2);

% Animation loop
for nT = 1:step_size:nT_max
    x = 1:nT;       % nT values up to the current point
    y = x.^3;       % nT^3
    
    % Update plot
    set(h, 'XData', x, 'YData', y);
    
    % Set axis limits dynamically
    % xlim([0 nT_max + 5]);
    % ylim([0 (nT_max)^3 + 100]);
    
    drawnow;
    pause(pause_time);
end

disp('Animation complete.');
