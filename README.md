# battery_model
Used to estimate the battery configurations we need.To explain the provided MATLAB script in a Markdown format, I'll break down each section and describe its purpose and functionality.

```markdown
## Pod Characteristics

Here, the script defines the physical and aerodynamic properties of the hyperloop pod, such as mass, drag coefficient, air density, friction coefficient, gravitational acceleration, frontal area, mass factor for rotating parts, and the gradient of inclination.

```matlab
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
```

## Mass Calculation

This section calculates the acceleration mass of the pod, taking into account the mass factor for rotating parts.

```matlab
m_acc = e_i1 * m_fzg + m_zul;
```

## Vehicle Dynamics Parameters

Defines initial, maximum, and end velocities, the total distance the pod must cover, and the diameter of the driving wheel.

```matlab
v_init = 0; % Starting Velocity in ms^-1
v_max = 60/3.6; % Maximum Velocity in ms^-1
v_end = 0; % End Velocity in ms^-1
sf = 150; % Pod Distance in m
Rd = 0.1; % Diameter of Driving wheel in m
```

## Motor Parameters

Specifies the peak power of the motor and its efficiency.

```matlab
P_peak = 37 * 1000; % Power in W at 7000 rpm
eff = 0.96;
P_motor = eff * P_peak;
```

## Resistance Calculation for Continuous Power

Calculates the forces acting against the pod: rolling resistance, air resistance, and gradient resistance.

```matlab
F_roll = f_r * m_fzg * g * cos(alpha * 180 / pi);
F_luft = 0.5 * c_L * rho_L * A * (v_max^2);
F_st = m_fzg * g * sin(alpha * 180 / pi);
```

## Maximum Acceleration Calculation

Determines the maximum possible acceleration of the pod given the net force available from the motor after overcoming resistances.

```matlab
a_max = ((P_motor/v_max) - F_roll - F_luft - F_st) / m_acc;
```

## S-Curve Generation

Generates the time segments for the S-curve which describes the pod's acceleration profile. The script calculates when to start and stop accelerating and decelerating to achieve a smooth velocity curve.

```matlab
T = (a_max/j_max) + (v_max/a_max) + (sf/v_max);
...
```

The coefficients for the S-curve's segments are calculated to define the acceleration, velocity, and position over time.

## Graph Generation

Generates plots for the distance, velocity, acceleration, and jerk over time using the calculated coefficients.

```matlab
...
%Plotting
figure(4)
subplot(4,1,1);
plot(t, s,'b','LineWidth', 3);
...
```

## Energy Demand

Calculates the energy demand for each segment of the pod's journey, integrating the power required over the time of travel.

```matlab
...
% Energy Demand Calculation
en_dem1 = integral(@(x) Pow_calc(x,a1,b1,c1,d1),0,t1);
...
energy_bed = en_dem1 + en_dem2 + en_dem3 + en_dem4;
energy_bed_wh = energy_bed * 0.000268; % in Wh
```

## Battery Parameters

Determines the required battery configuration (series and parallel) to meet the energy demand, considering the cells' voltage, capacity, C-rate, and state of charge limits.

```matlab
...
n_ser = ceil(V_bed/U_zell);
C_tot = energy_bed_wh/(n_ser*U_zell*eff_mech);
n_par = ceil(C_tot/(C_zell_ah * (SoC_max - SoC_min)/100));
...
```

In the end, the script provides the battery configuration needed for the hyperloop pod to complete its journey and calculates the number of possible runs based on the energy demand and battery capacity.
```

This breakdown provides a detailed understanding of each step in the MATLAB script, explaining how the script calculates various parameters essential for designing and analyzing the hyperloop pod's performance.
