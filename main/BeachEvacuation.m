
%=================================
%
%       BEACH EVACUATION
% 
%=================================
% Marcel Thielmann & Fabio Crameri

clear


%--------------------------------------------------------------------------
% run name
Parameter.Foldername        = 'BeachEvacuationTest';      % Run name
Parameter.Save              = logical(0);   % switch for saving output
Parameter.SaveTimeStep      = 5;          % saves every PlotTimeStep

%--------------------------------------------------------------------------
% setup parameters
StreetWidth = 5;
BuildingWidthX = 10;
BuildingWidthY = 12;
BeachWidth = 30;
BeachHeight = 5;
MaxElevation = 10;

%--------------------------------------------------------------------------
% numerical parameter
Parameter.resolution        = 0.2;          % resolution in [m]

Parameter.maxtime           = 3;            % maximum time to run in [min]
Parameter.dt                = 0.01;         % time step in [s]


%--------------------------------------------------------------------------
% number of agents
Parameter.nagent            = 1;           % number of agents

% agent parameters (can be perturbed with a random perturbation)
Parameter.AgentSetup        = 'random';     % 'random' 'given' 'load'
Parameter.m                 = 80;           % mass in kg
%Parameter.m_pert            = 10;

Parameter.v0                = 1;            % maximal/desired velocity [m/s]
Parameter.v0_pert           = 0;   

Parameter.t_acc             = 0.5;        	% acceleration time in [s]

Parameter.AgentSize         = 0.25;         % agent radius
%Parameter.AgentSize_pert    = 0.05; 

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
Parameter.ExitForce = 'constant';

%--------------------------------------------------------------------------
% flooding
Parameter.WithFlood         = true;        % true or false, determines if flood is taken into account for shortest path computation

Parameter.z0_flood          = 0;            % [m]
Parameter.dzdt_flood        = 0.01;         % rising speed of flood [m/s]
Parameter.dangerousDepth    = 0.2;          % [m]
Parameter.FloodSpeed        = Parameter.v0/2; % agents speed in shallow water [m/s]

Parameter.A_flood           = 5e2;          %[N]
Parameter.B_flood           = 0.08;         %[m]

%--------------------------------------------------------------------------
% shortest path computation
Parameter.DirectExitPath    = false;     	% fast marching algorithm (false); direct exit line (true)

Parameter.orig_sensitivity	= 1;            % best to leave this fixed
Parameter.agent_sensitivity	= 2.5;            % how much are agents taken into account when computing the shortest path? [0: not taken into account - high value: taken into account]
Parameter.topo_sensitivity 	= 1;            % sensitivity of agents to topography

Parameter.decision_time   	= 0.05;         % after which time does an agent redecide on its path?

Parameter.WithAgents        = true;        % true or false, determines if agents are taken into account for shortest path computation
Parameter.WithTopo          = true;        % true or false, determines if topography is taken into account for shortest path computation

Parameter.Enlarge           = 0.2;          % enlarge buildings for shortest path formulation


%--------------------------------------------------------------------------
% topography
Parameter.Topo_name         = 'BeachTopo';       % topography file to be loaded (if there is none, use 'none')

% parameters for slope-dependent velocity
Parameter.slope_f           = 3.5;
Parameter.slope_crit        = 0.05;


%--------------------------------------------------------------------------
% model domain
Parameter.xmin              = 0;
Parameter.xmax              = 110;
Parameter.ymin              = 0;
Parameter.ymax              = 110;

% create building list (if not given)


% create buildings on beach
x_min_build = Parameter.xmin + BeachWidth;
x_max_build = Parameter.xmax;
y_min_build = Parameter.ymin + BeachWidth;
y_max_build = Parameter.ymax-StreetWidth-4; % leave 4 meters for "dam"
DomainWidth = x_max_build-x_min_build;
DomainHeight = y_max_build-y_min_build;
BuildingList = SetBuildings(DomainWidth,DomainHeight,StreetWidth,BuildingWidthX,BuildingWidthY); % for beach houses

% shift building list values by real min values
BuildingList(:,1:2) = BuildingList(:,1:2) + x_min_build;
BuildingList(:,3:4) = BuildingList(:,3:4) + y_min_build;

BuildingList                = [ BuildingList                % coordinates of building xmin xmax ymin ymax
                                BeachWidth 100 Parameter.ymax-4 Parameter.ymax        % boundary wall
                                105 180 Parameter.ymax-4 Parameter.ymax       % boundary wall
                                185 200 Parameter.ymax-4 Parameter.ymax
                                
                                            ];
        
% create exit list (if not given)
ExitList                    = [                 % coordinates of exits: xmin xmax ymin ymax
                                100 105 Parameter.ymax-4 Parameter.ymax
                                180 185 Parameter.ymax-4 Parameter.ymax
                                            ];
       
% create starting area map for agents
StartingList                = [             	% coordinates of start area: xmin xmax ymin ymax
                                0 BeachWidth Parameter.ymin Parameter.ymax
                                0 Parameter.xmax Parameter.ymin BeachWidth
                                            ];
       
                                        
%--------------------------------------------------------------------------
%plotting parameters
Plotting.PlotEvolution      = true;             % direct plotting
Parameter.PlotTimeStep      = 1;               % plots every PlotTimeStep

Plotting.Marking            = 'none';           % 'none', 'number', 'smiley'
Plotting.FontSize           = 14;
Plotting.Color              = 'y';           % agents color: 'y' or [0 1 0]


%---------------------------------------
% test map
%---------------------------------------
% TestMap(Parameter,BuildingList,ExitList);
if ~strcmp(Parameter.Topo_name,'none')
    CreateTopo;
    save(Parameter.Topo_name,'XTopo','YTopo','ZTopo');
end
%---------------------------------------
% run simulation
%---------------------------------------
EscapePanic(Parameter,BuildingList,ExitList,StartingList,Plotting);






