function pos = forward_kinematics_func_num(q1,q2,q3,q4)
% a = q1;
% b = q2;
% c = q3;
% d = q4;
% syms q1;
% syms q2;
% syms q3;
% syms q4; 
x_fk = -14*cosd(q1)*sind(q2) - 14.8*cosd(q1)*cosd(q4)*sind(q2) - 14.8*sind(q4)*cosd(q3)*sind(q1) - 14.8*sind(q4)*cosd(q1)*cosd(q2)*sind(q3);
y_fk = -14*sind(q1)*sind(q2) - 14.8*cosd(q4)*sind(q1)*sind(q2) + 14.8*sind(q4)*cosd(q1)*cosd(q3) - 14.8*cosd(q2)*sind(q1)*sind(q3)*sind(q4);
z_fk = 2.7 + 14*cosd(q2) + 14.8*cosd(q2)*cosd(q4) - 14.8*sind(q2)*sind(q3)*sind(q4);
%fwd = forward_kinematics_func(1,1,1,1); %symbolic FK with dummy values 
%fwd_mat_value = eval(subs(fwd, {'q1','q2','q3','q4'}, {qo(1,1),qo(2,1),qo(3,1),qo(4,1)}));
%pos = double(subs(fwd,[q1,q2,q3,q4],[a,b,c,d]));
pos=[x_fk;y_fk;z_fk];
end