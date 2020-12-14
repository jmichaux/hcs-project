function [xout,yout,thetaout,vxout,vyout,vthetaout,axout,ayout,athetaout] = optimizeAcceleration(goalx,goaly)
% state index struct
idx_state = struct('x',     1,...
                   'y',     2,...
                   'th',    3,...
                   'vx',    4,...
                   'vy',    5,...
                   'vth',   6,...
                   'accx',  7,...
                   'accy',  8,...
                   'accth', 9,...
                   't',     10);
nz = length(fieldnames(idx_state));        % state dim

% Choose FRS from library according appropriate IC
FRS = load('Rcont_test');
nFRS = length(FRS.Rcont);  % 
c_IC = FRS.options.x0;
disp("Initializing params");

%% Create Obstacles
[Obstacles] = Create_Obstacles(idx_state);

disp("Obstacles created!");

%% First slice out known generators and add to center for each zonotope
% Output: Rcont_new = (c, <g_slice, g_nonslice>)
for i = 1:nFRS
    init_idx = idx_state.vx:idx_state.vy;
    init_val = c_IC(init_idx);
    FRS.Rcont_init_slice{i,1} = slice_zonotope(FRS.Rcont{i,1},init_idx,init_val);
end
disp("Initial Zonotope slice for position and velocity generators");

%% Extract Z_nonslice from Rcont_init_slice
idx_slice = idx_state.accx:idx_state.accy;
for i=1:nFRS
    Zi = FRS.Rcont_init_slice{i,1}.Z;
    ci = Zi(:,1);
    Gi = Zi(:,2:end);
    [~,g_slice_col] = find(Gi(idx_slice,:));
    
    G_slice = Gi(:,g_slice_col);
   
    Gi(:,g_slice_col) = [];
    G_nonslice = Gi;
    
    Z_slice = zonotope([ci,G_slice]);
    FRS.Rcont_slice{i,1} = Z_slice;
    
    Z_nonslice = zonotope([zeros(nz,1),G_nonslice]);
    FRS.Rcont_nonslice{i,1} = Z_nonslice;
%   k_slice = find(G(k_dim, :) ~= 0);
end
disp("Remove non-sliceable generators from Reachable set zonotopes.");
disp("Remaining Zonotope has only slceable generators");

%% Add nonslice zonotope to obstacles zonotope
nObs = Obstacles.nObs;
for i = 1:nObs
    for j = 1:nFRS
        Z_obs = Obstacles.zonotopes{i,1};
        Z_nonslice = FRS.Rcont_nonslice{j,1}.Z;
        c_ns = Z_nonslice(1:2,1);
        G_ns = Z_nonslice(1:2,2:end);
        Z_nonslice = zonotope([c_ns,G_ns]);
        
        Obstacles.buffered_zonotopes{j,i} = ...
            generateNewZonoObs(Z_obs,Z_nonslice);
        
        [Obstacles.Apoly{j,i},Obstacles.bpoly{j,i}] = polytope_PH(Obstacles.buffered_zonotopes{j,i}.Z);
    end
end
disp("Add nonsliceable zonotopes to obstacles.");

% for j = 1:nFRS
%    hold on; 
%    p = plotFilled(Obstacles.buffered_zonotopes{j,1},[1,2],'b');
%    p.FaceAlpha = 0; 
% end
%% OPT
% [A, b] means the polyhedron with nonslice generator for obstacles
% C means the new center of each zonotope
% G means the slice generator for each zonotope

% x_goal = 0;
% y_goal = 0;
% x_init = c_IC(idx_state.x);
% y_init = c_IC(idx_state.y);
% vx_init = c_IC(idx_state.vx);
% vy_init = c_IC(idx_state.vy);
% time_step = FRS.options.timeStep;

disp("Solving...");
% cvx_begin
% variables k(2,1) %
% minimize norm(x_init + vx_init*time_step + (1/2)*k(1,1)*time_step^2 - x_goal)+ ...
%          norm(y_init + vy_init*time_step + (1/2)*k(2,1)*time_step^2 - y_goal)
% for i = 1:nFRS
%     C = FRS.Rcont_slice{i,1}.center;
%     G = FRS.Rcont_slice{i,1}.Z(:,2:end);
%     C_new = C(1:2) + G(1:2,:)*k;
%     A = Obstacles.Apoly{i,1};
%     b = Obstacles.bpoly{i,1};
%     -max(A*C_new-b) <= 0;
% end
% cvx_end

%% Fmincon

disp('Solving FMincon');

k_guess = zeros(2,1);
k_UB = 2*ones(2,1);  % delta*(-)1 + c
k_LB = -k_UB;

x_goal = goalx;
y_goal = goaly;
X_goal = [x_goal; y_goal];

x_init = c_IC(idx_state.x);
y_init = c_IC(idx_state.y);
vx_init = c_IC(idx_state.vx);
vy_init = c_IC(idx_state.vy);


[sol,Cost_sol,flag] = fmincon(@(k) Cost_Func(k,c_IC,X_goal,idx_state,FRS),k_guess,[],[],[],[],...
k_LB,k_UB,@(k)Nonlinear_Con(k,idx_state,FRS,Obstacles,vx_init,vy_init));


% while (flag ~= 1)
% x_goal = 0.5*(x_init - (-3))+x_init;
% y_goal = 0.5*(y_init - (3))+y_init;
% [sol,Cost_sol,flag] = fmincon(@(k) Cost_Func(k,c_IC,X_goal,idx_state,FRS),k_guess,[],[],[],[],...
% k_LB,k_UB,@(k)Nonlinear_Con(k,idx_state,FRS,Obstacles,vx_init,vy_init));
% 
% end
% if (flag ~= 1)
%     return;
% end
% flag
% if(flag ~= 1)
%  tspan = 1;
%  x_new_end = 0.2*(x_init - (-2))+x_init;
%  y_new_end = 0.2*(y_init - (3))+y_init;
%  sol(1,1) = 2*(x_new_end - x_init - vx_init);
%  sol(2,1) = 2*(y_new_end - y_init - vy_init);
% end

%% Plots
% Original FRS
figure(1);
% for i = 1:nFRS
%    hold on; 
%    p = plotFilled(FRS.Rcont{i,1}{1},[1,2],'b');
%    p.FaceAlpha = 0;
% end

% Solution Trajectory
% x_init = c_IC(idx_state.x);
% y_init = c_IC(idx_state.y);
% vx_init = c_IC(idx_state.vx);
% vy_init = c_IC(idx_state.vy);
tspan = FRS.options.tFinal - FRS.options.tStart;
x_end = x_init + vx_init*tspan + (1/2)*sol(1,1)*tspan^2;
y_end = y_init + vy_init*tspan + (1/2)*sol(2,1)*tspan^2;
% if (y_end <= 0)
%     y_end = 0; 
%     sol(2,1) = 2*(y_end - y_init - vy_init*tspan)/tspan^2;   
% end

xout = x_end;
yout = y_end;
thetaout = 0;
vxout = vx_init+sol(1,1)*tspan;
vyout = vy_init+sol(2,1)*tspan;
vthetaout = 0;
axout = sol(1,1);
ayout = sol(2,1);
athetaout = 0;




% Obstacle
for i =1:nObs 
hold on;
pobs = plotFilled(Obstacles.zonotopes{i},[1,2],'r');
pobs.FaceAlpha = 0;
end

for i = 1:nFRS
    hold on;
   C_obj = FRS.Rcont_slice{i,1}.center;
    G_obj = FRS.Rcont_slice{i,1}.Z(:,2:end);
    delta_g_obj = [G_obj(idx_state.accx,1); G_obj(idx_state.accy,2)]; % change with accth
    lambda_obj = (sol - C_obj(7:8))./delta_g_obj;
    C_new_obj = C_obj(1:2) + G_obj(1:2,:)*lambda_obj;
    gs_obj = FRS.Rcont_nonslice{i,1}.Z(1:2,2:end);
    obj_zono = zonotope([C_new_obj,gs_obj]);
    p = plotFilled(obj_zono,[1,2],'b');
   p.FaceAlpha = 0;
   if (i == nFRS)
       C_new_end = C_obj + G_obj*lambda_obj;
       xout = C_new_end(1,1);
      yout = C_new_end(2,1);
      thetaout = C_new_end(3,1);
%     vxout = C_new_end(4,1);
%      vyout = C_new_end(5,1);
   end
end
scatter( xout, yout,50,'filled');

% Goal
% hold on;
% scatter(x_goal,y_goal,20,'filled');

% Plot settings
hold on; yline(0,'k');



%% Cost
% sol_cost = Cost_Func(sol,c_IC,X_goal,idx_state,FRS)
end