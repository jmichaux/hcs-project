function [Cost] = Cost_Func(k,c_IC,X_goal,idx_state,FRS)
x_goal = X_goal(1);
y_goal = X_goal(2);
x_init = c_IC(idx_state.x);
y_init = c_IC(idx_state.y);
vx_init = c_IC(idx_state.vx);
vy_init = c_IC(idx_state.vy);
tspan = FRS.options.tFinal - FRS.options.tStart;
x = x_init + vx_init*tspan + (1/2)*k(1,1)*tspan^2 - x_goal;
y = y_init + vy_init*tspan + (1/2)*k(2,1)*tspan^2 - y_goal;
delta_state = [x;
    y];

Cost = sum(delta_state.^2);


end