function AGENT = InitializeAgents(nagent,Parameter)

%==========================================================================
% initialize agents

% number and identifier
cell_array = num2cell(1:nagent);
[AGENT(1:nagent).name]      = cell_array{:};
[AGENT(1:nagent).num]       = cell_array{:};


% status
[AGENT(1:nagent).Status] 	= deal(1);   %[1: dry/ok  2: wet/ok]


% maximum velocity
v0          = Parameter.v0;
if isfield(Parameter,'v0_pert')
   v0_pert                  =  Parameter.v0_pert*2; % because only the amplitude is prescribed
   cell_array               =  num2cell(v0 + (rand(nagent,1)-0.5)*v0_pert);
   [AGENT(1:nagent).VMax]   =  cell_array{:};
else
    [AGENT(1:nagent).VMax]      = deal(v0);
end


% acceleration time
t_acc          = Parameter.t_acc;
if isfield(Parameter,'t_acc_pert')
   t_acc_pert                  =  Parameter.t_acc_pert*2;
   cell_array               =  num2cell(t_acc + (rand(nagent,1)-0.5)*t_acc_pert);
   [AGENT(1:nagent).t_acc]   =  cell_array{:};
else
    [AGENT(1:nagent).t_acc]      = deal(t_acc);
end


% mass
m          = Parameter.m;
if isfield(Parameter,'m_pert')
   m_pert                  =  Parameter.m_pert*2;
   cell_array               =  num2cell(m + (rand(nagent,1)-0.5)*m_pert);
   [AGENT(1:nagent).m]   =  cell_array{:};
else
    [AGENT(1:nagent).m]      = deal(m);
end


% agent radius
AgentSize          = Parameter.AgentSize;
if isfield(Parameter,'AgentSize_pert')
   AgentSize_pert                  =  Parameter.AgentSize_pert*2;
   cell_array               =  num2cell(AgentSize + (rand(nagent,1)-0.5)*AgentSize_pert);
   [AGENT(1:nagent).Size]   =  cell_array{:};
else
    [AGENT(1:nagent).Size]      = deal(AgentSize);
end

% agent box size
BoxSize          = Parameter.BoxSize;
if isfield(Parameter,'BoxSize_pert')
   BoxSize_pert                  =  Parameter.BoxSize_pert*2;
   cell_array               =  num2cell(BoxSize + (rand(nagent,1)-0.5)*BoxSize_pert);
   [AGENT(1:nagent).BoxSize]   =  cell_array{:};
else
    [AGENT(1:nagent).BoxSize]      = deal(BoxSize);
end


% agent velocity
[AGENT(1:nagent).Vel]       = deal(0); % initial velocity
[AGENT(1:nagent).VelX]      = deal(0); % initial velocity X
[AGENT(1:nagent).VelY]      = deal(0); % initial velocity Y

% direction vector
[AGENT(1:nagent).DirX]      = deal(1); % x-direction vector
[AGENT(1:nagent).DirY]      = deal(0); % y-direction vector

% force fields
[AGENT(1:nagent).FxPhysAgents]   = deal(0);
[AGENT(1:nagent).FyPhysAgents]   = deal(0);
[AGENT(1:nagent).FxPhysWall]     = deal(0);
[AGENT(1:nagent).FyPhysWall]     = deal(0);
[AGENT(1:nagent).FySocialAgents] = deal(0);
[AGENT(1:nagent).FySocialAgents] = deal(0);
[AGENT(1:nagent).FxSocialWalls]  = deal(0);
[AGENT(1:nagent).FySocialWalls]  = deal(0);
[AGENT(1:nagent).FxSocialFlood]  = deal(0);
[AGENT(1:nagent).FySocialFlood]  = deal(0);
[AGENT(1:nagent).xForceExit]     = deal(0);
[AGENT(1:nagent).yForceExit]     = deal(0);
[AGENT(1:nagent).LocX]           = deal(0);
[AGENT(1:nagent).LocY]           = deal(0);
[AGENT(1:nagent).LocZ]           = deal(0);
