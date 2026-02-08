%What is needed Lateral + heading
%PLACEHOLDERS (maybe right)
id_x = [1, 2, 6, 5, 4]; %phi, psi, r, p, beta
id_u = [8, 9]; %rudder, aileron
A_red = SS_lat_lo.A(id_x, id_x);
B_red = SS_lat_lo.A(id_x, id_u);
C_red = [0 0 1 0 0; 0 0 0 1 0; 0 0 0 0 1];%    r, p, beta
D_red = zeros(3,2);

sys_low = ss(A_red, B_red, C_red, D_red);

sys_low.StateName = {'phi', 'psi', 'r', 'p', 'beta'};
sys_low.InputName = {'aileron', 'rudder'};
sys_low.OutputName = {'r', 'p', 'beta'};