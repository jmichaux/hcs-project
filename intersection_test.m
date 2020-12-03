clear; close all;clc;

%% zonotope intersection

%create zonotopes
Z1 = zonotope([0 1 0; 0 0 1]);
Z2 = zonotope([1 1.5 0; 0 0 1.5]);
Z3 = zonotope([-3 1 0; 0 0 1]);


% display zonotopes
figure; hold on;
p = plotFilled(Z1, [1 2] ,'b');
p.FaceAlpha = 0.4;
p = plotFilled(Z2, [1 2], 'g');
p.FaceAlpha = 0.4;
p = plotFilled(Z3, [1 2], 'r');
p.FaceAlpha = 0.4;

% get zonotope arrays
z1 = Z1.Z;
z2 = Z2.Z;
z3 = Z3.Z;

% check intersection
% get center of first zonotope
c = z1(:,1);

% create buffered zonotope
zz2 = [z2 z1(:,2:end)];
zz3 = [z3 z1(:,2:end)];

% create half space representation of 
[A2, b2] = polytope_PH(zz2);
[A3, b3] = polytope_PH(zz3);

% check intersection
-max(A2*c - b2) < 0
-max(A3*c - b3) < 0