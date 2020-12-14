function [c, ceq] = Nonlinear_Con(k,idx_state,FRS,Obstacles,vx_init,vy_init)
c = [];
ceq = [0];
nFRS = length(FRS.Rcont);
nObs = Obstacles.nObs;
for i = 1:nObs
for j = 1:nFRS
    C = FRS.Rcont_slice{j,1}.center;
    G = FRS.Rcont_slice{j,1}.Z(:,2:end);
    delta_g = [G(idx_state.accx,1); G(idx_state.accy,2)]; % change with accth
    lambda = (k - C(7:8))./delta_g;
    C_new = C(1:2) + G(1:2,:)*lambda;
    A = Obstacles.Apoly{j,i};
    b = Obstacles.bpoly{j,i};
    %Add a small safe region around the obstacle
    c = [c; -max(A*C_new-b-0.1)];
end
end
%Add velocity constrains
c = [c;  -(vy_init+k(2,1)+0.5);(vy_init+k(2,1)-0.5);  -(vx_init+k(1,1)+0.2); (vx_init+k(1,1)-0.2)];
end