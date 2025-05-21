clc
clear
close all

% Parameters
Fs = 1000;               % Sampling frequency [Hz]
T = 1/Fs;                % Sampling period
L = 1000;                % Signal length
t = (0:L-1) * T;         % Time vector

f0 = 5;                  % Frequency of cosine wave [Hz]
signal_cos = cos(2*pi*f0*t);

% Exponential decay envelope
tau = 0.5;  % Time constant in seconds
envelope = exp(-t / tau);


% Multiply cosine with envelope
signal1 = signal_cos;


% Sweep settings
noise_levels = [0, 0.1, 0.3, 0.5, 1];  % Noise std dev
phase_shifts = 0:pi/6:pi;             % Phase shifts (radians)

% Store correlation
correlation_matrix = zeros(length(noise_levels), length(phase_shifts));

% Plot a few example signals
example_noise_levels = [0, 0.3, 1];
example_phase_shifts = [1, 2, 2];
example_sifts= [0, 0.5, 2];
figure('Name', 'Signal Comparisons');
plot_idx = 1;
startBool = true;
for ni = 1:length(example_noise_levels)
    for pi_ = 1:length(example_phase_shifts)
        noise_std = example_noise_levels(ni);
        scale_factor = example_phase_shifts(pi_);
        shift_factor = example_sifts(pi_);
        
        % Modify signal2 with scaling and noise
        noise = noise_std * randn(1, L);

        % Modify signal2 with scaling and noise
        
        if pi_ ==1
            signal2 = signal1 + noise;
            % startBool = false;
        elseif pi_ ==2
            signal2 = signal1 * scale_factor + shift_factor + noise;
            % startBool = false;
        else
            signal2 = envelope .* signal1 * scale_factor + shift_factor + noise;
        end

        % Pearson correlation
        r = corr(signal1', signal2')^2;

        signal_power = mean((signal1 * scale_factor).^2);
        noise_power = mean(noise.^2);
        SNR_dB = 10 * log10(signal_power / noise_power);


        % Calculate noise
        noise = signal2 - signal1;
        
        % Power of signal and noise
        signal_power = mean(signal1.^2);
        noise_power = mean(noise.^2);
        
        % Compute SNR in decibels
        SNR_dB2 = 10 * log10(signal_power / noise_power);


        % RMSE
        rmse = sqrt(mean((signal1 - signal2).^2));

        % Plot
        subplot(length(example_noise_levels), length(example_phase_shifts), plot_idx);
        plot(t, signal1, 'b', 'LineWidth', 1.2); hold on;
        plot(t, signal2, 'r', 'LineWidth', 1.2);
        legend('Signal 1', 'Signal 2');


        if pi_ ==1
            title(sprintf('\\sigma=%.1f  | R^2=%.2f | RMSE=%.2f', ...
                noise_std, r, rmse));
        elseif pi_ ==2
            title(sprintf('Scaled and offset & \\sigma=%.1f  | R^2=%.2f | RMSE=%.2f', ...
                noise_std, r, rmse));
        else
            title(sprintf('Scaled, offset and deformed & \\sigma=%.1f  | R^2=%.2f | RMSE=%.2f', ...
                noise_std, r, rmse));
        end
         % title(sprintf('Scale=%.1f & SNR=%.1f dB| R^2=%.2f | RMSE=%.2f| SNR2=%.1f dB', ...
         %    scale_factor, SNR_dB , r, rmse, SNR_dB2));
        xlabel('Time [s]'); ylabel('Amplitude');
        xlim([0 1]);
        plot_idx = plot_idx + 1;
    end
end

%%
% Now compute full correlation heatmap
for i = 1:length(noise_levels)
    for j = 1:length(phase_shifts)
        noise_std = noise_levels(i);
        phase_shift = phase_shifts(j);

        signal2 = cos(2*pi*5*t + phase_shift) + noise_std * randn(1, L);
        r = corr(signal1', signal2');
        correlation_matrix(i, j) = r;
    end
end

% Plot heatmap
figure('Name', 'Correlation Heatmap');
imagesc(rad2deg(phase_shifts), noise_levels, correlation_matrix);
%%

% Complexity levels
complexity = [0, 1, 3];

% MSE values
mse_sdp = [0.25875, 0.83962, 3.37469];
mse_orb = [0.27517, 1.96175, 1.29647];

% Pearson correlation values
pearson_sdp = [0.91485, 0.89172, 0.82082];
pearson_orb = [0.92704, 0.77516, 0.81045];

% Plot MSE
figure;
plot(complexity, mse_sdp, 'bo-', 'LineWidth', 1.5); hold on;
plot(complexity, mse_orb, 'ro-', 'LineWidth', 1.5);
xlabel('Complexity'); ylabel('MSE');
legend('SDP-GD', 'ORB-SLAM2');
title('Average MSE vs Complexity');
grid on;

% Plot Pearson Correlation
figure;
plot(complexity, pearson_sdp, 'bo-', 'LineWidth', 1.5); hold on;
plot(complexity, pearson_orb, 'ro-', 'LineWidth', 1.5);
xlabel('Complexity'); ylabel('Pearson Correlation');
legend('SDP-GD', 'ORB-SLAM2');
title('Average Pearson Correlation vs Complexity');
grid on;
