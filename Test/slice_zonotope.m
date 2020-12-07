function Z = slice_zonotope(Z, idx, val)
% SLICE_ZONOTOPE - slice a zonotope given a generator index and value for 
% its coefficient
assert(length(idx) == length(val));

% get zonotope as array
Z = Z{1}.Z;
c = Z(:,1);
G = Z(:,2:end);

% get sliceable generator
for i = 1:length(idx)
    idx_column = find(G(idx(i),:));
    g_col = G(:,idx_column);
    c = c + val(i)*g_col;  % add sliced generator to center
    
    % remove columns that were added to center
    G(:,idx_column) = [];
end

% Remove zero generators
G(:,~any(G)) = []; 

% return new zonotope
Z = [c G];
Z = zonotope(Z);
end

