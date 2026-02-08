%% AE4-301P Chapter 7: Pitch Rate Command System Design
clear; clc; close all;

% --- 1. SYSTEM DEFINITION (From Uploaded Image) ---
% Flight Condition Parameters
V_ft = 600;            % Velocity [ft/s]
V = V_ft  %* 0.3048;       % Velocity [m/s]
h_ft = 10000;          % Altitude [ft]
h = h_ft  %* 0.3048;      % Altitude [m]
g = 9.81 / 0.3048;         % Gravity acceleration [m/s^2]

% State Space Matrices (4-State Longitudinal: Vt, alpha, theta, q)
% Inputs: [Throttle, Elevator]
A_ac = [-0.0122  10.1727 -32.1700  -0.4813;
        -0.0002  -0.9006   0        0.9299;
         0        0        0        1.0000;
        -0.0000 -2.6378   0       -1.2238];

B_ac = [ 0.0016   0.1849;
        -0.0000  -0.0019;
         0        0;
         0       -0.1928];

C_ac = eye(4);
D_ac = zeros(4,2);

% --- 2. MODEL REDUCTION (Section 7.3, Step 1) ---
% Extract Short Period Model (States: alpha, q -> Indices 2, 4)
% Input: Elevator (Index 2)
A_sp = A_ac([2,4], [2,4]);
B_sp = B_ac([2,4], 2);
C_sp = eye(2);
D_sp = zeros(2,1);

sys_4state = ss(A_ac, B_ac(:,2), C_ac, D_ac(:,2)); % Elevator only
sys_sp_ol  = ss(A_sp, B_sp, C_sp, D_sp);



% Display Open Loop Eigenvalues
disp('--- Open Loop Dynamics ---');
damp(sys_sp_ol);
fprintf('\n');

dt = 0.001;
T = 30;
t = 0:dt:T;

% Input signal u(t): 1 for t < T_pulse, 0 otherwise
u = zeros(size(t));
u(t < 5) = -5;
u(t < 3) = 5;
u(t < 1) = 0;

% Simulate
figure('Name','Open Loop elevator Response');
[y_cl, t_cl] = lsim(sys_4state, u, t);
q_response = rad2deg(y_cl(:,4));
a_response = rad2deg(y_cl(:,2));


plot(t_cl, q_response, 'b', 'LineWidth', 1.5); hold on;
plot(t_cl, a_response, 'r--', 'LineWidth', 1.5);
title('Comparison of responses');
legend('q response', 'a - response');
xlabel('Time [s]'); ylabel('Pitch Rate [deg/s]');
grid on;

% --- 3. OPEN LOOP COMPARISON (Section 7.3, Step 2) ---
figure('Name','Step 2: Open Loop Comparison');
t = 0:0.01:5;
u = -1 * ones(size(t)); % Negative step (Pitch UP command)

% Simulate 4-state (Watch Output 4: q)
y_4st = lsim(sys_4state, u, t);
q_4st = rad2deg(y_4st(:,4));

% Simulate 2-state (Watch Output 2: q)
y_sp = lsim(sys_sp_ol, u, t);
q_sp = rad2deg(y_sp(:,2));

plot(t, q_4st, 'b', 'LineWidth', 1.5); hold on;
plot(t, q_sp, 'r--', 'LineWidth', 1.5);
title('Comparison of Pitch Rate (q) Response');
legend('4-State Model', '2-State Short Period Model');
xlabel('Time [s]'); ylabel('Pitch Rate [deg/s]');
grid on;

% --- 4. CONTROLLER DESIGN (Section 7.3, Step 3 & 4) ---
% Requirements
omega_req = 0.03 * V * 0.3048;       % V in [ft/s]   
zeta_req  = 0.5;              
T_theta2_inv_req = 0.75 * omega_req;

% Desired Poles
p1 = -zeta_req*omega_req + 1i*omega_req*sqrt(1-zeta_req^2);
p2 = -zeta_req*omega_req - 1i*omega_req*sqrt(1-zeta_req^2);
desired_poles = [p1; p2];

% Calculate Feedback Gains (K) using Ackermann's formula or place
% u = -Kx = -[K_alpha, K_q] * [alpha; q]
K = place(A_sp, B_sp, desired_poles);
K_alpha = K(1);
K_q = K(2);

fprintf('--- Controller Gains (Step 4) ---\n');
fprintf('Required Frequency: %.2f rad/s\n', omega_req);
fprintf('K_alpha: %.4f\n', K_alpha);
fprintf('K_q:     %.4f\n\n', K_q);

% --- 5. PRE-FILTER DESIGN (Section 7.3, Step 5) ---
% Calculate Aircraft Zero (1/T_theta2_ac)
% Approx numerator of q/de is: M_de*s + (M_alpha*Z_de - M_de*Z_alpha)
M_de = B_sp(2);
Z_de = B_sp(1);
M_alpha = A_sp(2,1);
Z_alpha = A_sp(1,1);

zero_ac = -(M_alpha*Z_de - M_de*Z_alpha) / M_de; % Algebraic zero
T_theta2_inv_ac = -zero_ac;

% Prefilter P(s) = (s + 1/T_theta2_req) / (s + 1/T_theta2_ac)
num_pre = [1, T_theta2_inv_req];
den_pre = [1, T_theta2_inv_ac];
sys_pre = tf(num_pre, den_pre);

fprintf('--- Pre-filter (Step 5) ---\n');
fprintf('Aircraft Zero (1/T_theta2): %.4f rad/s\n', T_theta2_inv_ac);
fprintf('Target Zero (1/T_theta2):   %.4f rad/s\n', T_theta2_inv_req);
fprintf('Prefilter P(s) = (s + %.2f) / (s + %.2f)\n\n', T_theta2_inv_req, T_theta2_inv_ac);

% --- 6. FEEDFORWARD GAIN (Section 7.3, Step 6) ---
% Calculate K_ff for unity DC gain
% Closed loop A matrix
A_cl = A_sp - B_sp*K;
% DC gain of (C * (sI - A_cl)^-1 * B * K_ff) must be 1/Prefilter_DC
% However, we want q_ss = r. 
% q is the 2nd state. C = [0 1].
C_out = [0 1]; 
DC_plant_cl = -C_out * inv(A_cl) * B_sp;
DC_pre = T_theta2_inv_req / T_theta2_inv_ac;
K_ff = 1 / (DC_plant_cl * DC_pre);

fprintf('--- Feedforward Gain (Step 6) ---\n');
fprintf('K_ff: %.4f\n\n', K_ff);

% --- 7. CLOSED LOOP VERIFICATION (Step 8) ---
% Create Closed Loop System
sys_cl_sp = ss(A_cl, B_sp*K_ff, C_sp, D_sp);
sys_total = sys_cl_sp * sys_pre; % Series connection

fprintf("Closed loop damping");
damp(sys_total);
fprintf("\n");

% Define Pulse Input (Block Input) [cite: 300, 317]
dt = 0.005;
T_pulse = 3.0; % Stick held for 3 seconds (enough to reach steady state)
T_end = 6.0;   % Simulation runs after stick release to measure dropback
t = 0:dt:T_end;

% Input signal u(t): 1 for t < T_pulse, 0 otherwise
u = zeros(size(t));
u(t < T_pulse) = 1;

% Simulate
figure('Name','Closed Loop Step Response');
[y_cl, t_cl] = lsim(sys_total, u, t);
q_response = y_cl(:,2);

% Extract responses
alpha_cl = y_cl(:,1);
q_cl = rad2deg(y_cl(:,2));
% Integrate q to get theta (approximate for short duration)
theta_cl = cumtrapz(t_cl, q_cl); 

subplot(2,1,1);
plot(t_cl, q_cl, 'LineWidth', 2); grid on;
yline(1, 'k--');
ylabel('Pitch Rate q [deg/s]'); title('Closed Loop Pitch Rate Tracking');

subplot(2,1,2);
plot(t_cl, theta_cl, 'r', 'LineWidth', 2); grid on;
ylabel('Pitch Attitude \theta [deg]'); xlabel('Time [s]');
title('Pitch Attitude Response');

% --- 8. CRITERIA PLOTS (Step 7) ---

% A. CAP CRITERION CHECK
% CAP = omega_sp^2 / (V/g * 1/T_theta2)
% Note: Using the REQUIRED values (since we forced them)
CAP_val = omega_req^2 / ( (V/g) * T_theta2_inv_req );

fprintf('--- Criteria Verification (Step 7) ---\n');
fprintf('Calculated CAP: %.4f\n', CAP_val);

%% CAP Criterion Visualization (Categories A, B & C)

% --- Setup Figure ---
figure('Name', 'CAP Criterion Analysis (All Categories)', 'Color', 'w');
hold on; grid on;
title('CAP Criterion: Flight Phase Categories A, B & C');
xlabel('Short Period Damping Ratio \zeta_{sp}');
ylabel('CAP [1/(g s^2)]');
set(gca, 'XScale', 'log', 'YScale', 'log');
axis([0.1 5 0.01 10]); % Zoomed slightly to fit relevant area

% --- CATEGORY C BOUNDARIES (Magenta) ---
% Category C: Terminal Flight Phases (Takeoff, Landing)
% Level 1 Region: zeta [0.35, 1.3], CAP [0.05, 3.6]
fill([0.35 1.3 1.3 0.35], [0.05 0.05 3.6 3.6], ...
     [1 0 1], 'FaceAlpha', 0.1, ...
     'EdgeColor', 'm', 'LineWidth', 2, 'LineStyle', '-', ...
     'DisplayName', 'Cat C: Terminal (Land/Takeoff)');

% --- CATEGORY B BOUNDARIES (Blue) ---
% Category B: Non-Terminal, Gradual Maneuvers (Cruise, Climb)
% Level 1 Region: zeta [0.30, 2.0], CAP [0.085, 3.6]
fill([0.30 2.0 2.0 0.30], [0.085 0.085 3.6 3.6], ...
     [0 0 1], 'FaceAlpha', 0.1, ...
     'EdgeColor', 'b', 'LineWidth', 2, 'LineStyle', '--', ...
     'DisplayName', 'Cat B: Cruise/Climb');

% --- CATEGORY A BOUNDARIES (Green) ---
% Category A: Rapid Maneuvering (Combat, Formation)
% Level 1 Region: zeta [0.35, 1.3], CAP [0.28, 3.6]
fill([0.35 1.3 1.3 0.35], [0.28 0.28 3.6 3.6], ...
     [0 1 0], 'FaceAlpha', 0.2, ...
     'EdgeColor', 'g', 'LineWidth', 3, ...
     'DisplayName', 'Cat A: Combat/Agile');

% --- Plot Design Point ---
plot(zeta_req, CAP_val, 'ro', 'MarkerSize', 12, 'LineWidth', 3, ...
     'MarkerFaceColor', 'r', 'DisplayName', 'Design Point');

% Add Text Label
text(zeta_req * 1.1, CAP_val, ...
     sprintf('  Design Point\n  (\\zeta=%.2f, CAP=%.2f)', zeta_req, CAP_val), ...
     'FontSize', 10, 'BackgroundColor', 'w', 'EdgeColor', 'k');

% Legend and Formatting
legend('Location', 'SouthEast', 'FontSize', 10);
grid on;

% B. GIBSON DROPBACK CHECK
% Calculate Dropback (DB) from time response
% Find peak pitch rate and steady state
[q_max, idx_max] = max(q_cl);
[theta_max, idtheta_max] = max(theta_cl);
q_ss = q_cl(T_pulse/dt);

% Find max pitch attitude (overshoot) or dropback
% Gibson defines DB via the pitch attitude response geometry
% Ideally: DB/q_ss = T_theta2 - 2*zeta/omega_n
DB_theoretical = (1/T_theta2_inv_req) - (2*zeta_req/omega_req);
DB_metric = DB_theoretical * q_ss; % Dimensional DB

% Gibson coordinates
theta_max
theta_cl(end)
q_ss
x_gibson = (theta_max-theta_cl(end))/q_ss;
y_gibson = DB_theoretical; % DB / q_ss

fprintf('Gibson Metric (DB/q_ss): %.4f sec\n', y_gibson);

figure('Name','Gibson Criterion');
hold on;
% Draw "Satisfactory" Region (Triangle from Fig 7.3)
patch([0, 0.3, 0.05], [1, 1, 3], 'c', 'FaceAlpha', 0.3);
plot(0, y_gibson, 'ro', 'MarkerSize', 10, 'LineWidth', 3);
text(0.02, y_gibson, '  Design Point');
xlim([-0.1 0.4]); ylim([1 3.5]);
xlabel('Smear (OS/q_s) [s]'); ylabel('Pitch Rate Overshoot (q_m/q_s)');
% X-axis: DB/q_s (positive right)
% Y-axis: q_m/q_s

clf;
hold on;
% Points approx: (0, 1) -> (0.3, 1) -> (0, 3)
patch([0 0.3 0.06 0], [1 1 3 3], 'c', 'FaceAlpha', 0.3);
plot(y_gibson, q_max/q_ss, 'ro', 'MarkerSize', 10, 'LineWidth', 3);
xlabel('Dropback (DB/q_s) [s]'); ylabel('Pitch Rate Overshoot Ratio (q_m/q_s)');
title('Gibson Dropback Criterion');
grid on;