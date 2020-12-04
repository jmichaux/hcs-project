function Z = slice_zonotope(Z, idx, val)
% SLICE_ZONOTOPE - slize a zonotope given a generator index and value for 
% its coefficient

% get zonotope as array
Z = Z.Z;
c = Z(:,1);
G = Z(:,2:end);

% get sliceable generator
g = G(:,idx);
G(:,idx) = [];

% add sliced generator to center
c = c + val*g;

% return new zonotope
Z = [c G];
Z = zonotope(Z);
end

