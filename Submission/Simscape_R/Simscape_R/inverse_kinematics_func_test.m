function qf = inverse_kinematics_func_test(qo,pos) 
%pos = [x(1);x(2);x(3)];
e = 10^(-2);
%qo = [0;20;-90;-100];
%initially
counter = 0;  
%J_inv_mat = inverse_jacobian_matrix(qo(1),qo(2),qo(3),qo(4));
J_f =[0 -0.100455858004140 0 -0.129154364647580;-0.307273208117348 0 -0.254384439280606 0;0 -0.307273208117348 0 0.223701921588887];
J_inv_mat = pinv(J_f);
fwd_mat_value = forward_kinematics_func_num(qo(1),qo(2),qo(3),qo(4));
fwd_mat_value = fwd_mat_value - pos;
q_new = qo - (J_inv_mat*fwd_mat_value);
error = abs(fwd_mat_value);

  while(error(1)>=e || error(2)>=e || error(3)>=e)
      q_n = q_new;
      %J_inv_mat = inverse_jacobian_matrix(q_n(1),q_n(2),q_n(3),q_n(4));
      J1 = [(7*pi*sin((pi*q_n(1))/180)*sin((pi*q_n(2))/180))/90 + (37*pi*cos((pi*q_n(4))/180)*sin((pi*q_n(1))/180)*sin((pi*q_n(2))/180))/450,- (7*pi*cos((pi*q_n(1))/180)*cos((pi*q_n(2))/180))/90 - (37*pi*cos((pi*q_n(1))/180)*cos((pi*q_n(2))/180)*cos((pi*q_n(4))/180))/450, 0, (37*pi*cos((pi*q_n(1))/180)*sin((pi*q_n(2))/180)*sin((pi*q_n(4))/180))/450;
    - (7*pi*cos((pi*q_n(1))/180)*sin((pi*q_n(2))/180))/90 - (37*pi*cos((pi*q_n(1))/180)*cos((pi*q_n(4))/180)*sin((pi*q_n(2))/180))/450, - (7*pi*cos((pi*q_n(2))/180)*sin((pi*q_n(1))/180))/90 - (37*pi*cos((pi*q_n(2))/180)*cos((pi*q_n(4))/180)*sin((pi*q_n(1))/180))/450, 0, (37*pi*sin((pi*q_n(1))/180)*sin((pi*q_n(2))/180)*sin((pi*q_n(4))/180))/450;
    0,                                   - (7*pi*sin((pi*q_n(2))/180))/90 - (37*pi*cos((pi*q_n(4))/180)*sin((pi*q_n(2))/180))/450, 0,                 -(37*pi*cos((pi*q_n(2))/180)*sin((pi*q_n(4))/180))/450];
J2 = [(37*pi*cos((pi*q_n(2))/180)*sin((pi*q_n(1))/180)*sin((pi*q_n(3))/180)*sin((pi*q_n(4))/180))/450 - (37*pi*cos((pi*q_n(1))/180)*cos((pi*q_n(3))/180)*sin((pi*q_n(4))/180))/450, (37*pi*cos((pi*q_n(1))/180)*sin((pi*q_n(2))/180)*sin((pi*q_n(3))/180)*sin((pi*q_n(4))/180))/450,   (37*pi*sin((pi*q_n(1))/180)*sin((pi*q_n(3))/180)*sin((pi*q_n(4))/180))/450 - (37*pi*cos((pi*q_n(1))/180)*cos((pi*q_n(2))/180)*cos((pi*q_n(3))/180)*sin((pi*q_n(4))/180))/450, - (37*pi*cos((pi*q_n(3))/180)*cos((pi*q_n(4))/180)*sin((pi*q_n(1))/180))/450 - (37*pi*cos((pi*q_n(1))/180)*cos((pi*q_n(2))/180)*cos((pi*q_n(4))/180)*sin((pi*q_n(3))/180))/450;
    - (37*pi*cos((pi*q_n(3))/180)*sin((pi*q_n(1))/180)*sin((pi*q_n(4))/180))/450 - (37*pi*cos((pi*q_n(1))/180)*cos((pi*q_n(2))/180)*sin((pi*q_n(3))/180)*sin((pi*q_n(4))/180))/450, (37*pi*sin((pi*q_n(1))/180)*sin((pi*q_n(2))/180)*sin((pi*q_n(3))/180)*sin((pi*q_n(4))/180))/450, - (37*pi*cos((pi*q_n(1))/180)*sin((pi*q_n(3))/180)*sin((pi*q_n(4))/180))/450 - (37*pi*cos((pi*q_n(2))/180)*cos((pi*q_n(3))/180)*sin((pi*q_n(1))/180)*sin((pi*q_n(4))/180))/450,   (37*pi*cos((pi*q_n(1))/180)*cos((pi*q_n(3))/180)*cos((pi*q_n(4))/180))/450 - (37*pi*cos((pi*q_n(2))/180)*cos((pi*q_n(4))/180)*sin((pi*q_n(1))/180)*sin((pi*q_n(3))/180))/450;
    0,                 -(37*pi*cos((pi*q_n(2))/180)*sin((pi*q_n(3))/180)*sin((pi*q_n(4))/180))/450,                                                                                    -(37*pi*cos((pi*q_n(3))/180)*sin((pi*q_n(2))/180)*sin((pi*q_n(4))/180))/450,                                                                                    -(37*pi*cos((pi*q_n(4))/180)*sin((pi*q_n(2))/180)*sin((pi*q_n(3))/180))/450];

J_f = J1+J2;
J_inv_mat = pinv(J_f);
      fwd_mat_value = forward_kinematics_func_num(q_n(1),q_n(2),q_n(3),q_n(4));
      fwd_mat_value = fwd_mat_value - pos;
      q_new = q_n - J_inv_mat*fwd_mat_value;
      error = abs(fwd_mat_value);
      counter=counter+1;
  end
 qf = q_new;
end


