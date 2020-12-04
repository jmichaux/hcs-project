clear;clc;

%% parameters
dim = 10; % [x; y; th; velx0; vely0; velth0; accx; accy; accth; t];
t_plan = 1;
dt = 0.1;

generate_xyth_dynamics()

VERBOSE = 1;

%% FRS Loop
for i = 1:1 % we're going to loop over all velocity intervals
    options.tStart = 0;
    options.tFinal = t_plan;
    
    % loop over intial velocities
    options.x0 = zeros(dim,1);
%     options.x0(4:5,1) = .5;
    options.x0(2,1) = 4;
    options.R0 = zonotope([options.x0, diag([0; 0; 0; 0.1; 0.1; 0; 0.1; 0.1; 0; 0])]);
    
    options.timeStep = dt;
    options.taylorTerms=5; %number of taylor terms for reachable sets
    options.zonotopeOrder= 2; %zonotope order... increase this for more complicated systems.
    options.maxError = 1000*ones(dim, 1);
    options.verbose = 1;
    
    options.uTrans = 0;
    options.U = zonotope([0, 0]);
    
    options.advancedLinErrorComp = 0;
    options.tensorOrder = 1;
    options.reductionInterval = inf;
    options.reductionTechnique = 'girard';
    
    %compute reachable set-----------------------------------------------------  
    sys = nonlinearSys(dim, 1, @lintraj, options);
    Rcont= reach(sys, options);

    % plotting
    if VERBOSE == 1
        dims=[1 2];

        figure;
        hold on

        %plot reachable sets
        for k=1:length(Rcont)
            plotFilled(Rcont{k}{1},dims,[.8 .8 .8],'EdgeColor','none');
        end

        %plot initial set
        plot(options.R0,dims,'w-','lineWidth',2);

        %plot simulation results
%             for k=1:length(simRes.t)
%                 plot(simRes.x{k}(:,dims(1)),simRes.x{k}(:,dims(2)),'Color',[0 0 0]);
%             end

        %label plot
        xlabel(['x_{',num2str(dims(1)),'}']);
        ylabel(['x_{',num2str(dims(2)),'}']);

    end

    filename = 'Rcont_test';
    save(filename, 'Rcont','options');
%     % save this FRS
%     my_c_IC = c_IC(i);
%     filename = sprintf('FRS_trig/trig_FRS_%0.3f.mat', my_c_IC);
%     save(filename, 'Rcont', 'options', 'L', 't_plan', 't_total', 'my_c_IC');
end