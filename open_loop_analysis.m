clear; clc; close all;

load('f16_trim_data_600fps.mat');

long_states = SS_long_lo.StateName;

idx_vt = find(strcmp(long_states, 'Vt'));
idx_alpha = find(strcmp(long_states, 'alpha'));
idx_theta = find(strcmp(long_states, 'theta'));
idx_q = find(strcmp(long_states, 'q'));

idx_thrust = find(strcmp(long_states, 'thrust_state'));
idx_elv = find(strcmp(long_states, 'elevator_state'));

rigid_indices = [idx_vt, idx_alpha, idx_theta, idx_q];
input_indices = [idx_thrust, idx_elv];

A_ac_lon = SS_long_lo.A(rigid_indices, rigid_indices);
B_ac_lon = SS_long_lo.A(rigid_indices, input_indices);
C_ac_lon = eye(4);
D_ac_lon = zeros(4, 2);

disp('--- Aircraft Matrices (Longitudinal) ---');
disp('A_ac (Longitudinal):'); disp(A_ac_lon);
disp('B_ac (Longitudinal):'); disp(B_ac_lon);

sys_lon = ss(A_ac_lon, B_ac_lon, C_ac_lon, D_ac_lon);
sys_lon.StateName = long_states(rigid_indices);
sys_lon.InputName = {'Thrust_cmd', 'Elevator_cmd'};
sys_lon.OutputName = sys_lon.StateName;

lat_states = SS_lat_lo.StateName;

idx_beta = find(strcmp(lat_states, 'beta'));
idx_phi = find(strcmp(lat_states, 'phi'));
idx_p = find(strcmp(lat_states, 'p'));
idx_r = find(strcmp(lat_states, 'r'));

idx_ail = find(strcmp(lat_states, 'aileron_state'));
idx_rud = find(strcmp(lat_states, 'rudder_state'));

lat_rigid_indices = [idx_beta, idx_phi, idx_p, idx_r];
lat_input_indices = [idx_ail, idx_rud];

A_ac_lat = SS_lat_lo.A(lat_rigid_indices, lat_rigid_indices);
B_ac_lat = SS_lat_lo.A(lat_rigid_indices, lat_input_indices);
C_ac_lat = eye(4);
D_ac_lat = zeros(4, 2);

disp('--- Aircraft Matrices (Lateral) ---');
disp('A_ac (Lateral):'); disp(A_ac_lat);
disp('B_ac (Lateral):'); disp(B_ac_lat);

sys_lat = ss(A_ac_lat, B_ac_lat, C_ac_lat, D_ac_lat);
sys_lat.StateName = lat_states(lat_rigid_indices);
sys_lat.InputName = {'Aileron_cmd', 'Rudder_cmd'};
sys_lat.OutputName = sys_lat.StateName;

a_value_check = SS_long_lo.A(idx_elv, idx_elv);
b_value_check = SS_long_lo.B(idx_elv, 2);

disp(['Servo A value (should be -20.2): ', num2str(a_value_check)]);
disp(['Servo B value (should be +20.2): ', num2str(b_value_check)]);

disp(' ');
disp('--- Longitudinal Modes (Short Period & Phugoid) ---');
[Wn_lon, Zeta_lon, Poles_lon] = damp(sys_lon);
T_half_lon = log(2) ./ abs(real(Poles_lon));
period_lon = 2*pi ./ abs(imag(Poles_lon));
damp(sys_lon)
disp(table(Poles_lon, period_lon, T_half_lon, ...
    'VariableNames', {'Pole', 'Period', 'T_half'}));

disp(' ');
disp('--- Lateral Modes (Dutch Roll, Roll, Spiral) ---');
[Wn_lat, Zeta_lat, Poles_lat] = damp(sys_lat);
T_half_lat = log(2) ./ abs(real(Poles_lat));
period_lat = 2*pi ./ abs(imag(Poles_lat));
damp(sys_lat)
disp(table(Poles_lat, period_lat, T_half_lat, ...
    'VariableNames', {'Pole', 'Period' 'T_half'}));

lineWidth = 1.5;
fontSize = 12;

% Short Period
[y_sp, t_sp] = initial(sys_lon, [0; 1*(pi/180); 0; 0], 0:0.01:5);

fig1 = figure('Name', 'Short Period', 'Position', [100, 100, 800, 500]);
plot(t_sp, y_sp(:,2)*(180/pi), 'LineWidth', lineWidth, 'DisplayName', '\alpha [deg]'); 
hold on;
plot(t_sp, y_sp(:,4)*(180/pi), 'LineWidth', lineWidth, 'DisplayName', 'q [deg/s]');
grid on; legend('Location', 'Best');
title('Short Period Mode'); xlabel('Time [s]'); ylabel('Amplitude [deg, deg/s]');
set(gca, 'FontSize', fontSize);

% Phugoid
[y_ph, t_ph] = initial(sys_lon, [10; 0; 0; 0], 0:0.1:600);

fig2 = figure('Name', 'Phugoid', 'Position', [150, 150, 800, 500]);
yyaxis left
plot(t_ph, y_ph(:,1), 'LineWidth', lineWidth, 'DisplayName', 'V_t [ft/s]');
ylabel('Velocity [ft/s]');
yyaxis right
plot(t_ph, y_ph(:,3)*(180/pi), 'LineWidth', lineWidth, 'DisplayName', '\theta [deg]');
ylabel('Pitch Angle [deg]');

grid on; legend('Location', 'Best');
title('Phugoid Mode'); xlabel('Time [s]');
set(gca, 'FontSize', fontSize);

% Dutch Roll
[y_dr, t_dr] = initial(sys_lat, [5*(pi/180); 0; 0; 0], 0:0.01:20);

fig3 = figure('Name', 'Dutch Roll', 'Position', [200, 200, 800, 500]);
plot(t_dr, y_dr(:,1)*(180/pi), 'LineWidth', lineWidth, 'DisplayName', '\beta [deg]');
hold on;
plot(t_dr, y_dr(:,2)*(180/pi), 'LineWidth', lineWidth, 'DisplayName', '\phi [deg]');
plot(t_dr, y_dr(:,4)*(180/pi), 'LineWidth', lineWidth, 'DisplayName', 'r [deg/s]');

grid on; legend('Location', 'Best');
title('Dutch Roll Mode'); xlabel('Time [s]'); ylabel('Amplitude [deg, deg/s]');
set(gca, 'FontSize', fontSize);


% Aperiodic Roll
[y_roll, t_roll] = initial(sys_lat, [0; 0; 10*(pi/180); 0], 0:0.01:4);

fig4 = figure('Name', 'Aperiodic Roll', 'Position', [250, 250, 800, 500]);
plot(t_roll, y_roll(:,3)*(180/pi), 'LineWidth', lineWidth, 'DisplayName', 'p [deg/s]');

grid on; legend('Location', 'Best');
title('Aperiodic Roll Mode'); xlabel('Time [s]'); ylabel('Roll Rate [deg/s]');
set(gca, 'FontSize', fontSize);

% Spiral Mode
[y_spi, t_spi] = initial(sys_lat, [0; 5*(pi/180); 0; 0], 0:0.1:200);

fig5 = figure('Name', 'Spiral Mode', 'Position', [300, 300, 800, 500]);
plot(t_spi, y_spi(:,2)*(180/pi), 'LineWidth', lineWidth, 'DisplayName', '\phi [deg]');

grid on; legend('Location', 'Best');
title('Spiral Mode'); xlabel('Time [s]'); ylabel('Bank Angle [deg]');
set(gca, 'FontSize', fontSize);