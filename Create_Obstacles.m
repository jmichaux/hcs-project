function [Obstacles] = Create_Obstacles(idx)
nEuc = 2;        % state dim
nObs = 4;
obs_zonotopes = cell(nObs,1);
centerX = [0 0 3 7]; % x dimension of the obstacle
centerY = [-10 3 4 8]; % y dimension of the obstacle
generator = [10 10;1 1;1 1;1 1]; % The generator for each obstacle is arranged in each row


% single obstacle
for i= 1:nObs
cobs = zeros(nEuc,1);
cobs(idx.x) = centerX(i);
cobs(idx.y) = centerY(i);
gobs = zeros(nEuc,1);
gobs(idx.x) = generator(i,1);
gobs(idx.y) = generator(i,2);
obs_zonotopes{i,1} = zonotope([cobs,diag(gobs)]);
end
% Build struct to output
Obstacles.zonotopes = obs_zonotopes;
Obstacles.nObs = nObs;
end