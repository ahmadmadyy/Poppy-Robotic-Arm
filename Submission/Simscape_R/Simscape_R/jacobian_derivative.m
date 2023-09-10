function J_dot = jacobian_derivative(q,q_dot)
%function J_dot = jacobian_derivative(q1,q2,q1d,q2d)
q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);
q1d = q_dot(1);
q2d = q_dot(2);
q3d = q_dot(3);
q4d = q_dot(4);
a = q1;
b = q2;
c = q3;
d = q4;
t = q1d;
u = q2d;
v = q3d;
r = q4d;
syms q1;
syms q2;
syms q3;
syms q4;
syms q1d;
syms q2d;
syms q3d;
syms q4d;
J = jacobian_matrix_symbolic(q1,q2,q3,q4);
h11 = q1d*diff(J(1,1),q1)+q2d*diff(J(1,1),q2)+q3d*diff(J(1,1),q3)+q4d*diff(J(1,1),q4);
h12 = q1d*diff(J(1,2),q1)+q2d*diff(J(1,2),q2)+q3d*diff(J(1,2),q3)+q4d*diff(J(1,2),q4);
h13 = q1d*diff(J(1,3),q1)+q2d*diff(J(1,3),q2)+q3d*diff(J(1,3),q3)+q4d*diff(J(1,3),q4);
h14 = q1d*diff(J(1,4),q1)+q2d*diff(J(1,4),q2)+q3d*diff(J(1,4),q3)+q4d*diff(J(1,4),q4);

h21 = q1d*diff(J(2,1),q1)+q2d*diff(J(2,1),q2)+q3d*diff(J(2,1),q3)+q4d*diff(J(2,1),q4);
h22 = q1d*diff(J(2,2),q1)+q2d*diff(J(2,2),q2)+q3d*diff(J(2,2),q3)+q4d*diff(J(2,2),q4);
h23 = q1d*diff(J(2,3),q1)+q2d*diff(J(2,3),q2)+q3d*diff(J(2,3),q3)+q4d*diff(J(2,3),q4);
h24 = q1d*diff(J(2,4),q1)+q2d*diff(J(2,4),q2)+q3d*diff(J(2,4),q3)+q4d*diff(J(2,4),q4);

h31 = q1d*diff(J(3,1),q1)+q2d*diff(J(3,1),q2)+q3d*diff(J(3,1),q3)+q4d*diff(J(3,1),q4);
h32 = q1d*diff(J(3,2),q1)+q2d*diff(J(3,2),q2)+q3d*diff(J(3,2),q3)+q4d*diff(J(3,2),q4);
h33 = q1d*diff(J(3,3),q1)+q2d*diff(J(3,3),q2)+q3d*diff(J(3,3),q3)+q4d*diff(J(3,3),q4);
h34 = q1d*diff(J(3,4),q1)+q2d*diff(J(3,4),q2)+q3d*diff(J(3,4),q3)+q4d*diff(J(3,4),q4);

%J_dot = [h11 h12 h13 h14;h21 h22 h23 h24;h31 h32 h33 h34];
J_dot_sym = [h11 h12 h13 h14;h21 h22 h23 h24;h31 h32 h33 h34];
J_dot = double(subs(J_dot_sym,[q1,q2,q3,q4,q1d,q2d,q3d,q4d],[a,b,c,d,t,u,v,r]));
%J_dot = diff(f1,q1)*q1d+diff(f1,q2)*q2d;
end
