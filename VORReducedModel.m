%What is needed Lateral + heading
%PLACEHOLDERS (maybe right)
id_x = [1, 2, 4, 5, 6];
id_u = [8, 9];
A_red = SS_lat_lo.A(id_x, id_x);
B_red = SS_lat_lo.A(id_x, id_u);
C_red = SS_lat_lo.C(id_x, id_x);
D_red = zeros(5,2);

sys_low = ss(A_red, B_red, C_red, D_red);

