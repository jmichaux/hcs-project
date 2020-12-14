clear all
ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss);
% load exampleMaps.mat
customizeMap = zeros(120,100);
customizeMap(81:101,1:11) = 1;
customizeMap(71:91,21:41) = 1;
customizeMap(31:51,61:81) = 1;
map = occupancyMap(customizeMap, 10);
sv.Map = map;
sv.ValidationDistance = 0.01;
ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
planner = plannerRRTStar(ss,sv);
planner.ContinueAfterGoalReached = true;
planner.MaxIterations = 2500;
planner.MaxConnectionDistance = 0.3;
start = [8, 10 0];
goal = [0, 0, 0];
rng(100, 'twister') % repeatable result
[pthObj, solnInfo] = plan(planner,start,goal);
map.show;
hold on;
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2), '.-'); % tree expansion
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2); % draw path