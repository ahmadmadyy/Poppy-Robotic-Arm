function q_dot = inverse_velocity_kinematics(q,V_F)
%J_inv = inverse_jacobian_matrix(q1,q2,q3,q4);
J = jacobian_matrix(q);
J_inv = pinv(J);
%q_dot = round(J_inv*V_F,0);
q_dot = J_inv*V_F;
end


