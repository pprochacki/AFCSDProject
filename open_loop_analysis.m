clear; clc; close all;

load('f16_trim_data_600fps.mat');


disp('--- Longitudinal Analysis ---')
long_states = SS_long_lo.StateName;
disp('Longitudinal States available:')
disp(long_states);

idx_vt = find(strcmp(long_states, 'Vt'));
idx_alpha = find(strcmp(long_states, 'alpha'));
idx_theta = find(strcmp(long_states, 'theta'));
idx_q = find(strcmp(long_states, 'q'));

idx_thrust = find(strcmp(long_states, 'thrust_state'));
idx_elv = find(strcmp(long_states, 'elevator_state'));

rigid_indices = [idx_vt, idx_alpha, idx_theta, idx_q];
input_indices = [idx_thrust, idx_elv];

A_lon = SS_long_lo.A(rigid_indices, rigid_indices);
B_lon = SS_long_lo.A(rigid_indices, input_indices);

sys_lon = ss(A_lon, B_lon, eye(4), 0);
sys_lon.StateName = long_states(rigid_indices);
sys_lon.InputName = {'Thrust_cmd', 'Elevator_cmd'};
sys_lon.OutputName = sys_lon.StateName;

disp('Longitudinal Eigenvalues:');
damp(sys_lon)

disp(' ');
disp('--- Lateral Analysi ---');
lat_states = SS_lat_lo.StateName;
disp('Lateral States available:');
disp(lat_states);

idx_beta = find(strcmp(lat_states, 'beta'));
idx_phi = find(strcmp(lat_states, 'phi'));
idx_p = find(strcmp(lat_states, 'p'));
idx_r = find(strcmp(lat_states, 'r'));

idx_ail = find(strcmp(lat_states, 'aileron_state'));
idx_rud = find(strcmp(lat_states, 'rudder_state'));

lat_rigid_indices = [idx_beta, idx_phi, idx_p, idx_r];
lat_input_indices = [idx_ail, idx_rud];

A_lat = SS_lat_lo.A(lat_rigid_indices, lat_rigid_indices);
B_lat = SS_lat_lo.A(lat_rigid_indices, lat_input_indices);

sys_lat = ss(A_lat, B_lat, eye(4), 0);
sys_lat.StateName = lat_states(lat_rigid_indices);
sys_lat.InputName = {'Aileron_cmd', 'Rudder_cmd'};
sys_lat.OutputName = sys_lat.StateName;

disp('Lateral Eignenvalues:');
damp(sys_lat)

a_value_check = SS_long_lo.A(idx_elv, idx_elv); % Should be approx -20.2
b_value_check = SS_long_lo.B(idx_elv, 2);        % Should be approx +20.2

disp(['Servo A value (should be -20.2): ', num2str(a_value_check)]);
disp(['Servo B value (should be +20.2): ', num2str(b_value_check)]);

figure;

step(sys_lon(4, 2), 10);
title('Pitch Rate (q) response to Elevator Step');
grid on;