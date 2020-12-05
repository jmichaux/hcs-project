function [c, ceq] = Nonlinear_Con(k,idx_state,FRS,Obstacles)
c = [];
ceq = [0];
nFRS = length(FRS.Rcont);
for i = 1:nFRS
    C = FRS.Rcont_slice{i,1}.center;
    G = FRS.Rcont_slice{i,1}.Z(:,2:end);
    delta_g = [G(idx_state.accx,1); G(idx_state.accy,2)]; % change with accth
    lambda = (k - C(7:8))./delta_g;
    C_new = C(1:2) + G(1:2,:)*lambda;
    A = Obstacles.Apoly{i,1};
    b = Obstacles.bpoly{i,1};
    c = [c; -max(A*C_new-b)];
    
end

end