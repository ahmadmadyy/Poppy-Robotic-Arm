function [J1,J2] = jacobian_matrix_split(q1,q2,q3,q4)
a = q1;
b = q2;
c = q3;
d = q4;
syms q1;
syms q2;
syms q3;
syms q4;
x1 = -14*cosd(q1)*sind(q2) - 14.8*cosd(q1)*cosd(q4)*sind(q2);
x2 = - 14.8*sind(q4)*cosd(q3)*sind(q1) - 14.8*sind(q4)*cosd(q1)*cosd(q2)*sind(q3);
y1 = -14*sind(q1)*sind(q2) - 14.8*cosd(q4)*sind(q1)*sind(q2);
y2 = 14.8*sind(q4)*cosd(q1)*cosd(q3) - 14.8*cosd(q2)*sind(q1)*sind(q3)*sind(q4);
z1 = 2.7 + 14*cosd(q2) + 14.8*cosd(q2)*cosd(q4);
z2 = - 14.8*sind(q2)*sind(q3)*sind(q4);
J1 = jacobian([x1,y1,z1],[q1,q2,q3,q4]);
J2 = jacobian([x2,y2,z2],[q1,q2,q3,q4]);
%J = double(subs(J_sym,[q1,q2,q3,q4],[a,b,c,d]));
end


