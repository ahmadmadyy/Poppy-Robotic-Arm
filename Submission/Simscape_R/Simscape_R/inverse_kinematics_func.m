function qf = inverse_kinematics_func(qo,pos) 
e = 10^(-2);
%initially
counter = 0;  
J_inv_mat = inverse_jacobian_matrix(qo);
fwd_mat_value = forward_kinematics_func_num(qo(1),qo(2),qo(3),qo(4));
% fwd_mat_value = [-14*cosd(qo(1))*sind(qo(2)) - 14.8*cosd(qo(1))*cosd(qo(4))*sind(qo(2)) - 14.8*sind(qo(4))*cosd(qo(3))*sind(qo(1)) - 14.8*sind(qo(4))*cosd(qo(1))*cosd(qo(2))*sind(qo(3));
%  -14*sind(qo(1))*sind(qo(2)) - 14.8*cosd(qo(4))*sind(qo(1))*sind(qo(2)) + 14.8*sind(qo(4))*cosd(qo(1))*cosd(qo(3)) - 14.8*cosd(qo(2))*sind(qo(1))*sind(qo(3))*sind(qo(4));
%  2.7 + 14*cosd(qo(2)) + 14.8*cosd(qo(2))*cosd(qo(4)) - 14.8*sind(qo(2))*sind(qo(3))*sind(qo(4))];
fwd_mat_value = fwd_mat_value - pos;
q_new = qo - (J_inv_mat*fwd_mat_value);
error = abs(fwd_mat_value);

  while(error(1)>=e || error(2)>=e || error(3)>=e)
      q_n = q_new;
      J_inv_mat = inverse_jacobian_matrix(q_n);
      fwd_mat_value = forward_kinematics_func_num(q_n(1),q_n(2),q_n(3),q_n(4));
      fwd_mat_value = fwd_mat_value - pos;
      q_new = q_n - J_inv_mat*fwd_mat_value;
      error = abs(fwd_mat_value);
      counter=counter+1;
  end
  qf = q_new;
end


