
%=================================
%
%       RUN ESCAPE PANIC
% 
%=================================
% Marcel Thielmann & Fabio Crameri

clear
%--------------------------------------------------------------------------
% run name
Parameter.Foldername        = 'test1';      % Run name
Parameter.SaveTimeStep      = 200;          % saves every PlotTimeStep


%--------------------------------------------------------------------------
% numerical parameter
Parameter.resolution        = 0.1;          % resolution in [m]

Parameter.maxtime           = 3;            % maximum time to run in [min]
Parameter.dt                = 0.01;         % time step in [s]


%--------------------------------------------------------------------------
% number of agents
Parameter.nagent            = 50;           % number of agents

% agent parameters (can be perturbed with a random perturbation)
Parameter.AgentSetup        = 'random';     % 'random' 'given' 'load'
Parameter.m                 = 80;           % mass in kg
Parameter.m_pert            = 10;

Parameter.v0                = 1;            % maximal/desired velocity [m/s]
Parameter.v0_pert           = 0;   

Parameter.t_acc             = 0.5;        	% acceleration time in [s]

Parameter.AgentSize         = 0.25;         % agent radius
Parameter.AgentSize_pert    = 0.05; 

Parameter.BoxSize           = 5;            % agent box size


%--------------------------------------------------------------------------
% physical forces parameters (Helbing,2000)
Parameter.PhysicalForces    = true;
Parameter.Tangential        = false;
Parameter.k                 = 1.2e5;        %1.2e5 [Helbing 2000]
Parameter.kappa             = 2.4e5;        %2.4e5 [Helbing 2000]

% social force parameters
Parameter.SocialForces      = true;
Parameter.pert_social       = 0.0;          % maximal amplitude factor of social agent forces perturbation e.g. 0.05
Parameter.A                 = 2e3;          %[N]  [2e3 Helbing 2000]
Parameter.B                 = 0.08;         %[m]  [0.08 Helbing 2000]
Parameter.ExitFactor        = 1;            %for adjusting strength of constant exit force field


%--------------------------------------------------------------------------
% shortest path computation
Parameter.DirectExitPath    = false;     	% fast marching algorithm (false); direct exit line (true)

Parameter.orig_sensitivity	= 3;            % best to leave this fixed
Parameter.agent_sensitivity	= 2;            % how much are agents taken into account when computing the shortest path? [0: not taken into account - high value: taken into account]
Parameter.topo_sensitivity 	= 1;            % sensitivity of agents to topography

Parameter.decision_time   	= 0.05;         % after which time does an agent redecide on its path?

Parameter.WithAgents        = false;        % true or false, determines if agents are taken into account for shortest path computation
Parameter.WithTopo          = false;        % true or false, determines if topography is taken into account for shortest path computation
Parameter.WithFlood         = false;        % true or false, determines if flood is taken into account for shortest path computation

Parameter.Enlarge           = 0.2;          % enlarge buildings for shortest path formulation


%--------------------------------------------------------------------------
% topography
Parameter.Topo_name         = 'none';       % topography file to be loaded (if there is none, use 'none')

% parameters for slope-dependent velocity
Parameter.slope_f           = 3.5;
Parameter.slope_crit        = 0.05;


%--------------------------------------------------------------------------
% model domain
Parameter.xmin              = 0;
Parameter.xmax              = 20;
Parameter.ymin              = 0;
Parameter.ymax              = 10;

% create building list (if not given)
BuildingList                = [                 % coordinates of building xmin xmax ymin ymax
                                0 16 0 1        % boundary wall
                                0 16 9 10       % boundary wall
                                15 16 3 9       % top barriere
                                10 11 1 7       % bottom barriere
                                            ];
        
% create exit list (if not given)
ExitList                    = [                 % coordinates of exits: xmin xmax ymin ymax
                                19 20 4.5 5.5
                                            ];
       
%--------------------------------------------------------------------------
%plotting parameters
Plotting.PlotEvolution      = true;             % direct plotting
Parameter.PlotTimeStep      = 50;               % plots every PlotTimeStep

Plotting.Marking            = 'none';           % 'none', 'number', 'smiley'
Plotting.FontSize           = 9;
Plotting.Color              = 'y';              % agents color





%---------------------------------------
% test map
%---------------------------------------
% TestMap(Parameter,BuildingList,ExitList);      
       
%---------------------------------------
% run simulation
%---------------------------------------
EscapePanic(Parameter,BuildingList,ExitList,Plotting);






