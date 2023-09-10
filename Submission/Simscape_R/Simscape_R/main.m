
%disp('20,40,50,-30')
%fwd_mat_value = eval(subs(fwd, {'q1','q2','q3','q4'}, {qo(1,1),qo(2,1),qo(3,1),qo(4,1)}))
%fwd_mat_value = eval(subs(fwd, {'q1','q2','q3','q4'}, {90,66.59,64.86,-159.11}))
q = inverse_kinematics_func([30;10;55;-70],[20.25;-1.399;4.411])
%J_inv = inverse_jacobian_matrix(20,40,50,-30)
%x = forward_kinematics_func_num(30,-50,70,-70)
%-10.49J1 = jacobian_derivative(30,40,50,-70,2,1,5,4)
%J1 = jacobian_derivative(q1,q2,q1d,q2d)