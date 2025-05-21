function [mainFrequencies, magnitudes, f, Z_mag] = extractMainHarmonics(t, z, minPeakRatio)
% extractMainHarmonics - Computes main harmonic frequencies of a signal
%
% Syntax:
%   [mainFrequencies, magnitudes, f, Z_mag] = extractMainHarmonics(t, z, minPeakRatio)
%
% Inputs:
%   t             - Timestamps (non-uniform sampling supported)
%   z             - Signal (same length as t)
%   minPeakRatio  - (Optional) Minimum peak height ratio (default = 0.1)
%
% Outputs:
%   mainFrequencies - Detected main harmonic frequencies (Hz)
%   magnitudes      - Corresponding amplitudes of detected peaks
%   f               - Frequency vector for plotting
%   Z_mag           - Magnitude spectrum

    if nargin < 3
        minPeakRatio = 0.1;
    end

    % Ensure column vectors
    t = t(:);
    z = z(:);

    % Resample to uniform time grid
    dt_avg = mean(diff(t));
    fs = 1 / dt_avg;                           % Approximate sampling rate
    t_uniform = linspace(t(1), t(end), numel(t));
    z_uniform = interp1(t, z, t_uniform);      % Linear interpolation

    % Detrend signal
    z_detrended = detrend(z_uniform);

    % FFT
    n = length(z_detrended);
    Z = fft(z_detrended);
    f = (0:n-1)*(fs/n);
    Z_mag = abs(Z)/n;
    Z_mag = Z_mag(1:floor(n/2));
    f = f(1:floor(n/2));

    % Find main harmonic frequencies
    [magnitudes, mainFrequencies] = findpeaks(Z_mag, f, ...
        'MinPeakHeight', max(Z_mag) * minPeakRatio);

    % Plot (optional - can be commented out)
    figure;
    plot(f, Z_mag);
    hold on;
    plot(mainFrequencies, magnitudes, 'rx');
    xlabel('Frequency (Hz)');
    ylabel('Amplitude');
    title('FFT Magnitude Spectrum with Main Harmonics');
    grid on;
end
