clear;

% domain
Parameter.xmin                = 0;
Parameter.xmax                = 20;
Parameter.ymin                = 0;
Parameter.ymax                = 10;
% number of agents
Parameter.nagent          	  = 30;      % number of agents

%numerical parameter
Parameter.resolution            = 0.1;      % resolution in [m]

Parameter.dt                	= 0.01;    	% time step in [s]
Parameter.maxtime               = 3;       % maximum time to run in [min]

Parameter.pert_social           = 0.05;     % maximal amplitude of social agent forces perturbation 
Parameter.decision_time         = 0.5;      % after which time does an agent redecide on its path?

% physical forces parameters (Helbing,2000)
Parameter.PhysicalForces    = true;
Parameter.Tangential        = false;
%Parameter.k                = 1.2e5;
Parameter.k                 = 1.2e7;
Parameter.kappa             = 2.4e5;

% social force parameters
Parameter.SocialForces   = true;
Parameter.A             = 2e3;   	%[N]  [2e3 Helbing 2000]
Parameter.B             = 0.08;     %[m]  [0.08 Helbing 2000]
Parameter.ExitFactor    = 3;        %for adjusting strength of constant exit force field

% shortest path computation
Parameter.agent_sensitivity               = 1; % how much are agents taken into account when computing the shortest path? [0: not taken into account - high value: taken into account]

% agent parameters (can be perturbed with a random perturbation)
Parameter.m                 = 80;       % mass in kg
Parameter.v0               	= 3;        % maximal/desired velocity [m/s]
Parameter.t_acc            	= 0.5;      % acceleration time in [s]
Parameter.AgentSize         = 0.3;      % agent radius
Parameter.BoxSize           = 5;      % agent box size

%---------------------------------------
% create building list (if not given)
%---------------------------------------
BuildingList = [
                 0 16 0 1    %boundary wall
                 0 16 9 10   %boundary wall
               %  6  7 1 7
               %  7  10 6 7
               %  12 13 3 9
                 
               %  9 12 3 4
               %  8 10 7 8
               % 9  11 2 4
                15 16 5 8   %top barriere
                15 16 1 4   %bottom barriere
                 
                ]; % coordinates of building xmin xmax ymin ymax
            

%---------------------------------------
% create exit list (if not given)
%---------------------------------------
ExitList = [
            
            19 20 4.5 5.5
            19 20  8 9]; % coordinates of exits: xmin xmax ymin ymax
       
       
       
%---------------------------------------
% run simulation
%---------------------------------------
EscapePanic(Parameter,BuildingList,ExitList);






