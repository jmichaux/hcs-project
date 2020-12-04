function [Obstacles] = Create_Obstacles(idx)
nEuc = 2;        % state dim
nObs = 1;
obs_zonotopes = cell(nObs,1);

% single obstacle
cobs1 = zeros(nEuc,1);
cobs1(idx.y) = 2;
gobs1 = zeros(nEuc,1);
gobs1(idx.x:idx.y) = 0.5;
obs_zonotopes{1,1} = zonotope([cobs1,diag(gobs1)]);

% Build struct to output
Obstacles.zonotopes = obs_zonotopes;
Obstacles.nObs = nObs;
end