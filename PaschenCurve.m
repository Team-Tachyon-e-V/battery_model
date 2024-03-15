% Constants for air
A = 112.5;  % [cm^-1 kPa]
B = 2737.5; % [V cm^-1 kPa]
gamma = 1.4;
C = log(A / log(1 + 1/gamma))  % Calculating C

% Fixed pressure in kPa
p = 100;  % Standard atmospheric pressure in kPa

% Converting pressure from Pa to Torr (1 Pa = 0.00750062 Torr)
p_Torr = p * 0.00750062;

% Varying distance between the electrodes in cm
d_cm = linspace(0.0, 0.1, 100);  % From "0.0" cm to 0.1 cm

% Calculating the pressure-distance product (pd) in Torr*cm
pd = p * d_cm;
%pd = linspace(10, 110, 1000);
% Initializing the breakdown voltage array
V_breakdown = zeros(1, length(pd));

% Calculating breakdown voltage using the provided formula over pd using a for loop
for i = 1:length(pd)
    V_breakdown(i) = B * pd(i) / (C + log(pd(i)));
end

% Plotting the Paschen curve over pd
figure;
plot(pd/p, V_breakdown, 'LineWidth', 2);
xlabel('Distance (cm)');
ylabel('Breakdown Voltage (V)');
title('Paschen Curve for Air at normal conditions');
grid on;
