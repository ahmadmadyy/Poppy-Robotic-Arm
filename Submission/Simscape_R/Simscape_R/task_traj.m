function Task_Space = task_traj(TF,Ts)
t=0;
while t<=TF
    if(t<=5)
        Task_Space = [-7.774-2.5952*t;0;17];
    else
        Task_Space = [-33.726+2.5952*t;0;17];
    end
t=t+Ts;
end
end

