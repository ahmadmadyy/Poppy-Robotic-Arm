function pos = forward_kinematics_func(q1,q2,q3,q4)
syms q1;
syms q2;
syms q3;
syms q4; 
T1 = Transformation_func(q1,2.7,0,90);
T2 = Transformation_func(q2,0,0,-90);
T3 = Transformation_func(q3+90,14,0,-90);
T4 = Transformation_func(q4-90,0,14.8,0);
TT = T1*T2*T3*T4;
%x = TT(1,4);
%y = TT(2,4);
%z = TT(3,4);
pos = [TT(1,4);TT(2,4);TT(3,4)];
end