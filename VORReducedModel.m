%What is needed Lateral + heading
%PLACEHOLDERS (maybe right)
id_x = [1, 2, 4, 6, 5]; %phi, psi, beta, r, p
id_u = [9, 8]; %rudder, aileron
A_red = SS_lat_lo.A(id_x, id_x);
B_red = SS_lat_lo.A(id_x, id_u);
C_red = SS_lat_lo.C([6,5,4], id_x);%    r, p, beta
D_red = zeros(3,2);

sys_low = ss(A_red, B_red, C_red, D_red);

