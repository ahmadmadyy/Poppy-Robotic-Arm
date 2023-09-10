function q = ang_calc(TF,Ts)
t=0;
qo = [0;20;-90;-100];
q = zeros(4,100);
i=1;
while t<=TF
    if(t<=5)
        Task_Space = [-7.938-3.0264*t;0;14];
        q(:,i) = inverse_kinematics_func_test(qo,Task_Space);
    else
        Task_Space = [-38.207+3.0264*t;0;14];
        q(:,i) = inverse_kinematics_func_test(qo,Task_Space);
    end
t=t+Ts;
i=i+1;
q
end