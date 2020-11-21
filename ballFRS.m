clear; close all;clc;

VERBOSE = 1;

% range of intial x-y velocities
x = linspace(0,10,10);
y = linspace(0,10,10);

x = 10;
y = 10;

% define linear system
A = [0 0 1 0;
     0 0 0 1;
     0 0 0 0;
     0 0 0 0];

B = [0;0;0;-9.81];

Abar = [A , B;zeros(1,5)];
Bbar = zeros(5,1);

% create the linear/nonlinear sys        
linSys = linearSys('linearSys', Abar, Bbar); 

%set options---------------------------------------------------------------
options.tStart = 0; %start time
options.tFinal = 2; %final time
options.timeStep = 0.01;
options.timeStepLoc{1} = 0.05; %time step size
options.taylorTerms = 10;
options.zonotopeOrder = 20;
options.polytopeOrder = 10;
options.errorOrder=2;
options.reductionTechnique = 'girard';
options.isHyperplaneMap = 0;
options.enclosureEnables = 5; %choose enclosure method(s)
options.originContained = 0;

for k = 1:length(x)
    for j =1:length(y)         
        % set input
        options.u = 1;
        options.uTrans = 0;
        options.U = zonotope([0, 0]);
        
        % choose zonotope
        options.x0 = [0; 0; x(k); y(j); 1]; %initial state
        options.R0 = zonotope([options.x0, 2*diag([0.05, 0.05, 0.05, 0.05, 0])]); %initial set
                
        % create reach set
        Rcont = reach(linSys, options); 
        
        % plotting
        if VERBOSE == 1
            dims=[1 2];

            figure;
            hold on

            %plot reachable sets
            for k=1:length(Rcont)
                plotFilled(Rcont{k},dims,[.8 .8 .8],'EdgeColor','none');
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
    end
end

