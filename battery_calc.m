clear all;

%% Pod Characteristics

m_fzg = 300; % Vehicle Mass in kg
m_zul = 0; % Payload
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

%% Vehicle Dynamics Parameters

v_init = 0; % Starting Velocity in ms^-1
v_max = 60/3.6; % Maximum Velocity in ms^-1
v_end = 0; % End Velocity in ms^-1
sf = 150; % Pod Distance in m
Rd = 0.1; % Diameter of Driving wheel in m

%% Motor Parameters 

P_peak = 37 * 1000; % Power in W at 7000 rpm
eff = 0.96;

P_motor = eff * P_peak;

%% Resistance Calculation for continuous power

F_roll = f_r * m_fzg * g * cos(alpha * 180 / pi);
F_luft = 0.5 * c_L * rho_L * A * (v_max^2);
F_st = m_fzg * g * sin(alpha * 180 / pi);

%F_acc = (e_i1 * m_fzg + m_zul) * a_max
%F_bed = F_roll + F_luft + F_st + F_acc

%% Maximum Accleration Calculation

a_max = ((P_motor/v_max) - F_roll - F_luft - F_st) / m_acc;

%% Declaration of other variables

j_max = 15; %in m/s^3
eff_mech = 0.85;

%% S-curve generation

T = (a_max/j_max) + (v_max/a_max) + (sf/v_max);

t1 = a_max/j_max;
t3 = (a_max/j_max) + (v_max/a_max);
t2 = t3 - t1;
t4 = T - t3;
t5 = T - t2;
t6 = T - t1;
t7 = T;


if t3 < T/2 && t4 > t3 && t5>t4 && t6>t5
    a1 = j_max;
    a2 = 0;
    a3 = -j_max;
    a4 = 0;
    a5 = -j_max;
    a6 = 0;
    a7 = j_max;
    
    b1 = 0;
    b2 = a_max;
    b3 = a_max + (j_max*t2);
    b4 = 0;
    b5 = j_max*t4;
    b6 = -a_max;
    b7 = -a_max - (j_max*t6);
    
    c1 = 0;
    c2 = (((a1*t1^2)/2) + (b1*t1) + c1) - (((a2*t1^2)/2) + (b2*t1));
    c3 = (((a2*t2^2)/2) + (b2*t2) + c2) - (((a3*t2^2)/2) + (b3*t2));
    c4 = (((a3*t3^2)/2) + (b3*t3) + c3) - (((a4*t3^2)/2) + (b4*t3));
    c5 = (((a4*t4^2)/2) + (b4*t4) + c4) - (((a5*t4^2)/2) + (b5*t4));
    c6 = (((a5*t5^2)/2) + (b5*t5) + c5) - (((a6*t5^2)/2) + (b6*t5));
    c7 = - (((a7*t7^2)/2) + (b7*t7));
    
    d1 = 0;
    d2 = (((a1*t1^3)/6) + ((b1*t1^2)/2) + (c1*t1) + d1) - (((a2*t1^3)/6) + ((b2*t1^2)/2) + (c2*t1));
    d3 = (((a2*t2^3)/6) + ((b2*t2^2)/2) + (c2*t2) + d2) - (((a3*t2^3)/6) + ((b3*t2^2)/2) + (c3*t2));
    d4 = (((a3*t3^3)/6) + ((b3*t3^2)/2) + (c3*t3) + d3) - (((a4*t3^3)/6) + ((b4*t3^2)/2) + (c4*t3));
    d5 = (((a4*t4^3)/6) + ((b4*t4^2)/2) + (c4*t4) + d4) - (((a5*t4^3)/6) + ((b5*t4^2)/2) + (c5*t4));
    d6 = (((a5*t5^3)/6) + ((b5*t5^2)/2) + (c5*t5) + d5) - (((a6*t5^3)/6) + ((b6*t5^2)/2) + (c6*t5));
    d7 = (((a6*t6^3)/6) + ((b6*t6^2)/2) + (c6*t6) + d6) - (((a7*t6^3)/6) + ((b7*t6^2)/2) + (c7*t6));

else
    disp('Please reiterate the values and give them');
    return;
end

%% Graph Generation

N = 1000;
t = linspace(0,T,N);

ind1 = find((0<=t) & (t<=t1));
ind2 = find((t1<t) & (t<=t2));
ind3 = find((t2<t) & (t<=t3));
ind4 = find((t3<t) & (t<=t4));
ind5 = find((t4<t) & (t<=t5));
ind6 = find((t5<t) & (t<=t6));
ind7 = find((t6<t) & (t<=t7));

%Distance Curve
s1 = ((a1*t.^3)/6) + ((b1*t.^2)/2) + (c1*t) + d1;
s2 = ((a2*t.^3)/6) + ((b2*t.^2)/2) + (c2*t) + d2;
s3 = ((a3*t.^3)/6) + ((b3*t.^2)/2) + (c3*t) + d3;
s4 = ((a4*t.^3)/6) + ((b4*t.^2)/2) + (c4*t) + d4;
s5 = ((a5*t.^3)/6) + ((b5*t.^2)/2) + (c5*t) + d5;
s6 = ((a6*t.^3)/6) + ((b6*t.^2)/2) + (c6*t) + d6;
s7 = ((a7*t.^3)/6) + ((b7*t.^2)/2) + (c7*t) + d7;

s = [s1(ind1), s2(ind2), s3(ind3), s4(ind4), s5(ind5), s6(ind6), s7(ind7)];

%Velocity Curve
v1 = a1*t.^2/2 + b1*t + c1;
v2 = a2*t.^2/2 + b2*t + c2;
v3 = a3*t.^2/2 + b3*t + c3;
v4 = a4*t.^2/2 + b4*t + c4;
v5 = a5*t.^2/2 + b5*t + c5;
v6 = a6*t.^2/2 + b6*t + c6;
v7 = a7*t.^2/2 + b7*t + c7;

v = [v1(ind1), v2(ind2), v3(ind3), v4(ind4), v5(ind5), v6(ind6), v7(ind7)];

%Acceleration Curve
acc1 = a1*t + b1;
acc2 = a2*t + b2;
acc3 = a3*t + b3;
acc4 = a4*t + b4;
acc5 = a5*t + b5;
acc6 = a6*t + b6;
acc7 = a7*t + b7;

acc = [acc1(ind1), acc2(ind2), acc3(ind3), acc4(ind4), acc5(ind5), acc6(ind6), acc7(ind7)];

%Jerk Curve

len = ones(1,length(t));%For multiplication with a value to get the curve

j1 = a1*len;
j2 = a2*len;
j3 = a3*len;
j4 = a4*len;
j5 = a5*len;
j6 = a6*len;
j7 = a7*len;

j = [j1(ind1), j2(ind2), j3(ind3), j4(ind4), j5(ind5), j6(ind6), j7(ind7)];

%Plotting

figure(4)
subplot(4,1,1);
plot(t, s,'b','LineWidth', 3);
xlim([0, T]);
xlabel('t (sec)');
ylabel('s (m)');

subplot(4,1,2);
plot(t, v,'r','LineWidth', 3);
xlim([0, T]);
xlabel('t (sec)');
ylabel('v (m/s)');

subplot(4,1,3);
plot(t, acc,'g','LineWidth', 3);
xlim([0, T]);
xlabel('t (sec)');
ylabel('acc (m/s^2)');

subplot(4,1,4);
plot(t, j,'b','LineWidth', 3);
xlim([0, T]);
xlabel('t (sec)');
ylabel('jerk (m/s^3)');

%% Energy Demand

vel = @(x,a,b,c) a.*(x.^2)./2 + b*x + c;
accl = @(x,a,b) a*x + b;

F_roll_calc = f_r * m_fzg * g * cos(alpha * 180 / pi);
F_luft_calc = @(x,a,b,c) 0.5 * c_L * rho_L * A .* (vel(x,a,b,c).^2);
F_st_calc = m_fzg * g * sin(alpha * 180 / pi);
F_acc_calc = @(x,a,b) (e_i1 * m_fzg + m_zul) .* accl(x,a,b);
F_bed_calc = @(x,a,b,c,d) F_roll_calc + F_luft_calc(x,a,b,c) + F_st_calc + F_acc_calc(x,a,b);
Pow_calc = @(x,a,b,c,d) F_bed_calc(x,a,b,c,d) .* vel(x,a,b,c);

% Segment 1
en_dem1 = integral(@(x) Pow_calc(x,a1,b1,c1,d1),0,t1);

% Segment 2
en_dem2 = integral(@(x) Pow_calc(x,a2,b2,c2,d2),t1,t2);

% Segment 3
en_dem3 = integral(@(x) Pow_calc(x,a3,b3,c3,d3),t2,t3);

% Segment 4
en_dem4 = integral(@(x) Pow_calc(x,a4,b4,c4,d4),t3,t4);

energy_bed = en_dem1 + en_dem2 + en_dem3 + en_dem4;
energy_bed_wh = energy_bed * 0.000268; % in Wh

%% Voltage and Current Need(from data sheet)

V_bed = 490; %in Voltage
I_bed = 100; %in Amps

%% Battery Parameters

U_zell = 4.2; %in Voltage
C_zell_ah = 3.1; %in Ah
C_rate = 35;

SoC_max = 100; % in %
SoC_min = 40; % in %


n_ser = ceil(V_bed/U_zell);
C_tot = energy_bed_wh/(n_ser*U_zell*eff_mech);
n_par = ceil(C_tot/(C_zell_ah * (SoC_max - SoC_min)/100));
C_rate_calc = I_bed / (n_par * C_zell_ah);

if C_rate_calc < C_rate
    energy_avail = n_ser * n_par * U_zell * C_zell_ah * (SoC_max - SoC_min) /100;
    num_of_runs = eff_mech * energy_avail / energy_bed_wh;
    X = sprintf('In Series: %d ; In Parallel: %d; Number of Runs: %.2f', n_ser, n_par, num_of_runs);
    disp(X)
else
    disp('You will not get the full performance of motor')
    return;
end