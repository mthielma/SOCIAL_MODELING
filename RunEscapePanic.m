clear;


% Workflow control
Parameter.DirectExitPath = true; % true or false, determines if the fast marching algorithm is used for shortest path computation (false), or if the exit direction is the direct line (true)

Parameter.WithAgents = false; % true or false, determines if agents are taken into account for shortest path computation
Parameter.WithTopo   = false; % true or false, determines if topography is taken into account for shortest path computation
Parameter.WithFlood  = false; % true or false, determines if flood is taken into account for shortest path computation

% subfolder and topo information
Foldername = 'test'; % subfolder where output is to be stored
Topo_name  = 'none'; % topography file to be loaded (if there is none, use 'none')

% domain
Parameter.xmin                = 0;
Parameter.xmax                = 20;
Parameter.ymin                = 0;
Parameter.ymax                = 10;
% number of agents
Parameter.nagent          	  = 50;      % number of agents

%numerical parameter
Parameter.resolution            = 0.1;      % resolution in [m]

Parameter.dt                	= 0.01;    	% time step in [s]
Parameter.maxtime               = 3;       % maximum time to run in [min]

Parameter.pert_social           = 0.05;     % maximal amplitude factor of social agent forces perturbation
Parameter.decision_time         = 0.01;      % after which time does an agent redecide on its path?

% physical forces parameters (Helbing,2000)
Parameter.PhysicalForces    = true;
Parameter.Tangential        = false;
Parameter.k                 = 1.2e7;
Parameter.kappa             = 2.4e5;

% social force parameters
Parameter.SocialForces   = true;
Parameter.A             = 2e3;   	%[N]  [2e3 Helbing 2000]
Parameter.B             = 0.08;     %[m]  [0.08 Helbing 2000]
Parameter.ExitFactor    = 1;        %for adjusting strength of constant exit force field

% shortest path computation
Parameter.orig_sensitivity    = 3; % best to leave this fixed
Parameter.agent_sensitivity   = 2; % how much are agents taken into account when computing the shortest path? [0: not taken into account - high value: taken into account]
Parameter.topo_sensitivity    = 1; % sensitivity of agents to topography

% parameters for slope-dependent velocity
Parameter.slope_f = 3.5;
Parameter.slope_crit = 0.05;


% agent parameters (can be perturbed with a random perturbation)
Parameter.m                 = 80;       % mass in kg
Parameter.m_pert            = 10;
Parameter.v0               	= 5;        % maximal/desired velocity [m/s]
Parameter.v0_pert           = 1;   

Parameter.t_acc            	= 0.5;      % acceleration time in [s]

Parameter.AgentSize         = 0.25;      % agent radius
Parameter.AgentSize_pert    = 0.05; 

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
                15 16 5.5 9   %top barriere
                15 16 1 4.5   %bottom barriere
                 
                ]; % coordinates of building xmin xmax ymin ymax
            

%---------------------------------------
% create exit list (if not given)
%---------------------------------------
ExitList = [
            
            19 20 4.5 5.5
%            19 20  8 9
            ]; % coordinates of exits: xmin xmax ymin ymax
       
       
       
%---------------------------------------
% run simulation
%---------------------------------------
EscapePanic(Parameter,BuildingList,ExitList,Foldername,Topo_name);






