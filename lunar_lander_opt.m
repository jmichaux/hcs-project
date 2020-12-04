%% Initialize
clear; clc;

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
c_IC = zeros(nz,1);
FRS = load('Rcont_test');
nFRS = length(FRS.Rcont);  % 

disp("Initializing params");

%% Create Obstacles
[Obstacles] = Create_Obstacles(idx_state);
disp("Obstacles created!");

%% First slice out known generators and add to center for each zonotope
% Output: Rcont_new = (c, <g_slice, g_nonslice>)
for i = 1:nFRS
    init_idx = idx_state.vx:idx_state.vth;
    init_val = c_IC(init_idx);
    FRS.Rcont_init_slice{i,1} = slice_zonotope(FRS.Rcont{i,1},init_idx,init_val);
end
disp("Initial Zonotope slice for position and velocity generators");

%% Extract g_nonslice from Rcont_new to be added to polytopes in opt
% Output: Rcont_new = (c, <gslice>)
%         Obs_new = (Aobs*g_nonslice, b_obs)





%% Final prep before opt
% formulate the center for optimization
% c_opt_i = c_IC + K*gslice   % c_opt for each zonotpe in the FRS



%% OPT
% [A, b] means the polyhedron with nonslice generator for obstacles
% C means the new center of each zonotope
% G means the slice generator for each zonotope
cvx_begin
x_goal = 0;
y_goal = 0;
variables k(2) % 
minimize norm(x_initial + vx_initial*time_step + (1/2) *k(1) *time_step-x_goal)+ ...
         norm(y_initial + vy_initial*time_step + (1/2) *k(2) *time_step-y_goal)
for i = 1:numZonotope
    C = generateNewCenter();
    G = generateSlicedGenerator();
    C_new = C + k'*G;
    [A, b] = generateNewObs();
    -max(A*C_new-b) <= 0;
end
cvx_end
















