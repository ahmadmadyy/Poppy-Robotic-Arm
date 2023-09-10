function J_sym = jacobian_matrix_symbolic(q1,q2,q3,q4)
syms q1;
syms q2;
syms q3;
syms q4;
x_fk = -14*cosd(q1)*sind(q2) - 14.8*cosd(q1)*cosd(q4)*sind(q2) - 14.8*sind(q4)*cosd(q3)*sind(q1) - 14.8*sind(q4)*cosd(q1)*cosd(q2)*sind(q3);
y_fk = -14*sind(q1)*sind(q2) - 14.8*cosd(q4)*sind(q1)*sind(q2) + 14.8*sind(q4)*cosd(q1)*cosd(q3) - 14.8*cosd(q2)*sind(q1)*sind(q3)*sind(q4);
z_fk = 2.7 + 14*cosd(q2) + 14.8*cosd(q2)*cosd(q4) - 14.8*sind(q2)*sind(q3)*sind(q4);
J_sym = jacobian([x_fk,y_fk,z_fk],[q1,q2,q3,q4]);
end


