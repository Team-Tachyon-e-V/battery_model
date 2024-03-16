clear all;

%% Pod Characteristics

m_fzg = 200; % Vehicle Mass in kg
m_zul = 50; % Payload
c_L = 0.8; % Drag Coefficient
rho_L = 1.2; % Density of air (kgm^-3)
f_r = 0.02; % Kinetic Friction Coefficient
g = 9.81; % Gravity in ms^-2
A = 1.5 * 0.3; % Area in m^2
e_i1 = 1.2; % Mass factor of rotating part
gradient = 5; % Inclination Gradient Percentage
alpha = atan(gradient/100); % Inclination Gradient Angle in radian

%% Mass Calculation

m_acc = e_i1 * m_fzg + m_zul;

%% Vehicle Dynamics Parameters - Parameterized acceleration

a_max = 5.4; % Maximum Acceleration in m/s^2 (Parameter)
a_dec = -6.944; % Fixed Deceleration in m/s^2

v_init = 0; % Starting Velocity in ms^-1
v_max = 20/3.6; % Maximum Velocity in ms^-1


v_end = 0; % End Velocity in ms^-1
sf = 50; % Pod Distance in m
xPod = 2;
sf = sf - xPod;


%% S-curve generation based on the acceleration and deceleration parameters

% Time to reach maximum velocity
t_acc_max = (v_max - v_init) / a_max;
s_acc_max = 0.5 * a_max * t_acc_max^2;

% Time to decelerate to zero velocity from maximum velocity
t_dec_zero = -v_max / a_dec;
s_dec_zero = 0.5 * a_dec * t_dec_zero^2 + v_max * t_dec_zero;

% Ensure the total distance covers the acceleration and deceleration phases
if (s_acc_max + s_dec_zero) > sf
    disp('Acceleration and deceleration distances exceed total distance. Adjust the parameters.');
    return;
end

% Calculate the cruising time and distance
s_cruise = sf - (s_acc_max + s_dec_zero);
t_cruise = s_cruise / v_max;

% Total time for the trip
T = t_acc_max + t_cruise + t_dec_zero;

%% Velocity, Distance, and Acceleration profile generation

% Generating time vector
N = 1000;
t = linspace(0, T, N);
dt = t(2) - t(1); % Time step

% Initializing velocity, distance, and acceleration profiles
v = zeros(1, N);
s = zeros(1, N);
a = zeros(1, N);

% Generating profiles
for i = 2:length(t)
    if t(i) <= t_acc_max
        v(i) = a_max * t(i);
        a(i) = a_max;
    elseif t(i) <= (t_acc_max + t_cruise)
        v(i) = v_max;
        a(i) = 0;
    else
        v(i) = v_max + a_dec * (t(i) - t_acc_max - t_cruise);
        a(i) = a_dec;
    end
    % Distance calculation by numerical integration of velocity
    s(i) = s(i-1) + v(i) * dt;
end

% Plotting
figure;
subplot(3,1,1);
plot(t, s, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Distance (m)');
title('Distance Profile');
grid on;

subplot(3,1,2);
plot(t, v, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Velocity Profile');
grid on;

subplot(3,1,3);
plot(t, a, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Acceleration Profile');
grid on;

