%=================================
% 
%          Escape Panic
% 
%=================================
% Marcel Thielmann & Fabio Crameri

% Comments:
% - agents initially placed too close to walls become rockets! ...fc
% 

clear;

%numerical parameter
resolution       	= 0.1;      % resolution in [m]
dt                	= 0.01;    	% time step in [s]
maxtime          	= 3;       % maximum time to run in [min]
pert                = 0.05;     % maximal amplitude of social agent forces perturbation of 
decision_time       = 0.1;      % after which time does an agent redecide on its path?
decision_step       = round(decision_time/dt);
agent_sensitivity   = 0.1; % reduce velocity by which factor if agent is on node?

%physical parameter
nagent          	= 30;      % number of agents

noUSEatPresent   	= logical(0);
SocialForce        	= logical(1);   %switch for social force

% physical forces parameters (Helbing,2000)
%Parameter.k         = 1.2e5;
Parameter.k         = 1.2e7;
Parameter.kappa  	= 2.4e5;

% social force parameters
Parameter.A       	= 2e3;   	%[N]  [2e3 Helbing 2000]
Parameter.B       	= 0.08;      %[m]  [0.08 Helbing 2000]
Parameter.ExitFactor= 1e6;   %for adjusting strength of constant exit force field

% agent parameters
m                 	= 80;       % mass in kg
v0               	= 3;        % maximal/desired velocity [m/s]
cutoffVelocity   	= logical(1); %sets maximum velocity at v0
t_acc            	= 0.5;      % acceleration time in [s]


addpath ./DecisionStrategy/
addpath ./WallForces/
addpath ./Plotting/
addpath ./kdtree_alg_OSX/
addpath ./FastMarching_version3b, add_function_paths();
%==========================================================================
% initialize fine grid (if not given as argument)
xmin                = 0;
xmax                = 20;
ymin                = 0;
ymax                = 10;

xvec                = xmin:resolution:xmax;
yvec                = ymin:resolution:ymax;
[X_Grid,Y_Grid]     = meshgrid(xvec,yvec);

% set topography
Z_Grid = 0*(0.01.*X_Grid+0.08.*Y_Grid);

% compute gradient
[Gradient_x,Gradient_y] = gradient(Z_Grid,resolution,resolution);

%convert time
maxtime = maxtime*60; %[min] => [s]

%==========================================================================
% initialize buildings
% - location
% - number of people inside
% - number of families inside
% - location of the exit

%---------------------------------------
% create starting area map for agents
%---------------------------------------
StartArea = zeros(size(yvec,2),size(xvec,2));
StartArea(find(X_Grid<5)) = 1;

%---------------------------------------
% create boundary map for later use
%---------------------------------------
BoundaryMap = zeros(size(yvec,2),size(xvec,2));
BoundaryMap(1,:)=1; BoundaryMap(size(yvec,2),:)=1; BoundaryMap(:,1)=1; BoundaryMap(:,size(xvec,2))=1;


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
            
            
            
            
            

BuildingList(find(BuildingList(:,1)>=xmax),:) = []; %if building fully outside domain: remove it!
BuildingList(find(BuildingList(:,3)>=ymax),:) = []; %if building fully outside domain: remove it!
BuildingList(find(BuildingList(:,2)>xmax),2) = xmax; %adjust building to domain boundary
BuildingList(find(BuildingList(:,4)>ymax),2) = ymax; %adjust building to domain boundary

BuildingMap = X_Grid*0;
% add buildings to map
for i=1:size(BuildingList,1)
    BuildingMap(X_Grid>=BuildingList(i,1) & X_Grid<=BuildingList(i,2) & Y_Grid>=BuildingList(i,3) & Y_Grid<=BuildingList(i,4)) = 1;
end

%---------------------------------------
% create exit list (if not given)
%---------------------------------------
ExitList = [
            
            19 20 4.5 5.5
           ]; % coordinates of exits: xmin xmax ymin ymax

Xexit = [(ExitList(:,2)+ExitList(:,1))/2]; 
Yexit = [(ExitList(:,3)+ExitList(:,4))/2]; 
       
       
ExitList(find(ExitList(:,1)>=xmax),:)   = []; %if exit fully outside domain: remove it!
ExitList(find(ExitList(:,3)>=ymax),:)   = []; %if exit fully outside domain: remove it!
ExitList(find(ExitList(:,2)>xmax),2)    = xmax; %adjust exit to domain boundary
ExitList(find(ExitList(:,4)>ymax),2)    = ymax; %adjust exit to domain boundary

ExitMap = X_Grid*0;
% add exits to map
for i=1:size(ExitList,1)
    ExitMap(X_Grid>=ExitList(i,1) & X_Grid<=ExitList(i,2) & Y_Grid>=ExitList(i,3) & Y_Grid<=ExitList(i,4)) = 1;
end

%[Dgradx,Dgrady] = ComputeShortestPathGlobal(BuildingList,Xexit,Yexit,X_Grid,Y_Grid,Gradient_x,Gradient_y,v0,resolution);

%----------------------------------------------------
% compute forces from buildings (static)
%----------------------------------------------------
[ForceX,ForceY] = ArchitectureForceV2(X_Grid,Y_Grid,BuildingList,Parameter,resolution);
xArchForces     = ForceX;
yArchForces     = ForceY;

% xArchForces = zeros(size(xvec,2),size(yvec,2));  %initialise force field x-direction (nx*ny)
% yArchForces = zeros(size(xvec,2),size(yvec,2));  %initialise force field y-direction (nx*ny)
% 
% ArchGeometry	= BuildingMap;
% ARCH.format     = 'map';                    %'list' or 'map'
% ARCH.type       = 1;                        %1: repulsive / 2: attractive
% Spreading       = {'exp' 'linear' 'const'};
% ARCH.spreading	= Spreading{1};
% ARCH.force      = 1.0;                      %1 is the same as wall force
% 
% [xArchForces_walls, yArchForces_walls, xArchDir_walls, yArchDir_walls] = f_RepWalls_single (X_Grid, Y_Grid, ArchGeometry, ARCH, Parameter);
% 
% %add contribution of object(s)
% xArchForces = xArchForces + xArchForces_walls;
% yArchForces = yArchForces + yArchForces_walls;
% 
% checkFigure = logical(1);
% if checkFigure
%     figure(11)
%     quiver(X_Grid',Y_Grid',xArchForces,yArchForces)
%     title('architecture force')
%     xlabel('x [m]')
%     ylabel('y [m]')
%     axis equal; axis tight 
% end

% %----------------------------------------------------
% % compute forces from exits (static)
% %----------------------------------------------------
% ArchGeometry	= ExitList;
% ARCH.format     = 'list';                   %'list' or 'map'
% ARCH.type    	= 2;                        %1: repulsive / 2: attractive
% Spreading       = {'exp' 'linear' 'const'};
% ARCH.spreading  = Spreading{3};
% ARCH.force      = 0.2;                      %1 is the same as wall force
% 
% [xArchForces_exits, yArchForces_exits, xArchDir_exits, yArchDir_exits] = f_RepWalls_single (X_Grid, Y_Grid, ArchGeometry, ARCH, Parameter);
% 
% %add contribution of object(s)
% xArchForces = xArchForces + xArchForces_exits;
% yArchForces = yArchForces + yArchForces_exits;

checkFigure = logical(1);
if checkFigure
    figure(11)
    quiver(X_Grid,Y_Grid,xArchForces,yArchForces)
    title('architecture force')
    xlabel('x [m]')
    ylabel('y [m]')
    axis equal; axis tight 
end

%==========================================================================
% initialize agents in buildings and on streets
% - location
% - mass
% - age -> maximum velocity, max acceleration
% - panic factor (how often are new strategies taken, how much are persons in the way influencing the cost of that path?)
% - horizon distance (how far can the agent look?)
% - group (e.g. family) --> later
% - helping factor --> later
cell_array = num2cell(1:nagent);
[AGENT(1:nagent).name]      = cell_array{:};
[AGENT(1:nagent).num]       = cell_array{:};
[AGENT(1:nagent).VMax]      = deal(v0);
[AGENT(1:nagent).BoxSize]   = deal(5); % the agent is only influenced by other agents inside this box
[AGENT(1:nagent).Vel]       = deal(0); % initial velocity (to be randomized)

[AGENT(1:nagent).VelX]      = deal(0); % initial velocity X
[AGENT(1:nagent).VelY]      = deal(0); % initial velocity Y

[AGENT(1:nagent).DirX]      = deal(1); % x-direction vector
[AGENT(1:nagent).DirY]      = deal(0); % y-direction vector

[AGENT(1:nagent).FxSoc]  	= deal(0); % initial social force X
[AGENT(1:nagent).FySoc] 	= deal(0); % initial social force Y
[AGENT(1:nagent).FxArch] 	= deal(0); % initial architecture force X
[AGENT(1:nagent).FyArch]  	= deal(0); % initial architecture force Y

VelX                        = num2cell([AGENT(1:nagent).Vel].*[AGENT(1:nagent).DirX]); % initial velocity X
VelY                        = num2cell([AGENT(1:nagent).Vel].*[AGENT(1:nagent).DirY]); % initial velocity Y


[AGENT(1:nagent).VelX]      = VelX{:};
[AGENT(1:nagent).VelY]      = VelY{:};


cell_array = num2cell((0.5+ (rand(nagent,1))*0.15)./2);
[AGENT(1:nagent).Size]   = cell_array{:};

% [AGENT(1:nagent).Size]   = deal(0.5/2);
% 
% AGENT(1).LocX = 3;
% AGENT(1).LocY = 5+0.2; 
% 
% AGENT(2).LocX = 3;
% AGENT(2).LocY = 5-0.2;

% get the indices in BuildingMap that correspond to a road
[iy,ix] = find(StartArea==1 & BoundaryMap==0 & BuildingMap==0 & ExitMap==0);
NoRoad  = length(iy);
% create random indices (make sure that there are no duplicates)
indices = randperm(NoRoad);

iyStart = iy(indices(1:nagent));
ixStart = ix(indices(1:nagent));

indices = sub2ind(size(X_Grid),iyStart,ixStart);

StartLocX = X_Grid(indices);
StartLocY = Y_Grid(indices);

cellX = num2cell(StartLocX);
cellY = num2cell(StartLocY);
[AGENT(1:nagent).LocX]       = cellX{:};
[AGENT(1:nagent).LocY]       = cellY{:};

% precompute factor for slope-dependent maximum velocity
PreFac = ([AGENT(1:nagent).VMax]'./exp(-3.5*0.05));

%----------------------------------------------------
% building locations
%----------------------------------------------------
x_Buildings = X_Grid(BuildingMap==1);
y_Buildings = Y_Grid(BuildingMap==1);

% plot setup
figure(1),clf
hold on
% plot buildings
PlotBuildings(BuildingList,'r');
PlotBuildings(ExitList,'g');
% plot agents
PlotAgents(nagent,AGENT,'y');
axis equal
axis([min(X_Grid(:)) max(X_Grid(:)) min(Y_Grid(:)) max(Y_Grid(:))])
box on
title('time = 0.00 min')
xlabel('x [m]')
ylabel('y [m]')

%==========================================================================
% time loop

time=0; itime=0;
while (time <= maxtime && size(AGENT,2)>0)
    time = time+dt;     %actual time
    itime = itime+1;    %nr. timesteps
    disp('*****************************************')
    disp(['timestep ',num2str(itime),':    time = ',num2str(time/60),' min'])
    
    if (nagent~=size(AGENT,2)); error('fc: nagent not equal nr. of agents!'); end

    %----------------------------------------------------
    % compute kdtree of agents for later use
    %----------------------------------------------------
    ReferencePoints         = zeros(nagent,2);
    ReferencePoints(:,1)    = [AGENT(:).LocX];
    ReferencePoints(:,2)    = [AGENT(:).LocY];
    
    % generate tree
    tree =   kdtree(ReferencePoints);
    
    % clearvars ReferencePoints
    
    %----------------------------------------------------
    % compute forces from buildings on all agents (just interpolate the
    % precomputed force field to the agents)
    %----------------------------------------------------
    [FxSocialWalls,FySocialWalls]         = ComputeSocialForcesStatic(AGENT,X_Grid,Y_Grid,xArchForces,yArchForces,Parameter);
    
    dummy                                 = num2cell(FxSocialWalls);
    [AGENT(1:nagent).FxSocialWalls]       = dummy{:};
    dummy                                 = num2cell(FySocialWalls);
    [AGENT(1:nagent).FySocialWalls]       = dummy{:};
    
    %----------------------------------------------------
    % compute direction field to exits on all agents 
    % (just interpolate the precomputed field to the agents)
    %----------------------------------------------------
    if (mod(itime,decision_step)==0 || itime==1)
        [Dgradx,Dgrady] = ComputeShortestPathGlobalWithAgents(BuildingList,Xexit,Yexit,X_Grid,Y_Grid,Gradient_x,Gradient_y,v0,resolution,AGENT,nagent,agent_sensitivity);
        xExitDirAgents = interp2(X_Grid,Y_Grid,Dgradx,[AGENT.LocX],[AGENT.LocY],'*linear');
        yExitDirAgents = interp2(X_Grid,Y_Grid,Dgrady,[AGENT.LocX],[AGENT.LocY],'*linear');
        
        dummy = num2cell(xExitDirAgents);
        [AGENT(1:nagent).xExitDir]       = dummy{:};
        dummy = num2cell(yExitDirAgents);
        [AGENT(1:nagent).yExitDir]       = dummy{:};
    end
    
    %----------------------------------------------------------------
    % compute the distance of all agents to the building polygons
    %----------------------------------------------------------------
    
    
%     for i = 1:size(BuildingList,1)
%         % generate polygon data for buildings
%         x(1) = BuildingList(i,1);
%         x(2) = BuildingList(i,1);
%         x(3) = BuildingList(i,2);
%         x(4) = BuildingList(i,2);
%         
%         y(1) = BuildingList(i,3);
%         y(2) = BuildingList(i,4);
%         y(3) = BuildingList(i,4);
%         y(4) = BuildingList(i,3);
%         
%         [d,x_poly,y_poly] = p_poly_dist([AGENT.LocX],[AGENT.LocY],x,y);
%     end
    
    
%     % set the destination direction (later on, this can be done in a more sophisticated manner)
%     DistExitX = XExit([AGENT.DestinationNode])-[AGENT.LocX];
%     DistExitY = YExit([AGENT.DestinationNode])-[AGENT.LocY];
%     DistExitTot = sqrt(DistExitX.*DistExitX+DistExitY.*DistExitY);
%     
%     xexitdir = num2cell(DistExitX./DistExitTot);
%     [AGENT(1:nagent).xExitDir] = xexitdir{:};
%     yexitdir = num2cell(DistExitY./DistExitTot);
%     [AGENT(1:nagent).yExitDir] = yexitdir{:};
%     % ExitForceX = Parameter.ExitFactor*Parameter.A.*DistExitX./DistExitTot;
%     % ExitForceY = Parameter.ExitFactor*Parameter.A.*DistExitY./DistExitTot;

    %----------------------------------------------------
    % agent loop
    iagent=0; nagent2 = nagent;
    for iagent = 1:nagent2
        x_agent         = AGENT(iagent).LocX;
        y_agent         = AGENT(iagent).LocY;
        agent_size      = AGENT(iagent).Size;
        velx_agent = AGENT(iagent).VelX;
        vely_agent = AGENT(iagent).VelY;
        %-------------------------------------------------
        % check if the agent is outside the domain
        %-------------------------------------------------
        if x_agent>xmax || x_agent<xmin || y_agent>ymax || y_agent<ymin
           error('fc: stupid agent outside domain!') 
        end
        
        %-------------------------------------------------
        % get the agents that are in the "individual box" and compute the
        % distance to them
        %-------------------------------------------------

        % generate the Boxes per Agent
        Boxes       = zeros(2,2);
        Boxes(1,1)  = [AGENT(iagent).LocX]-[AGENT(iagent).BoxSize]./2;
        Boxes(2,1)  = [AGENT(iagent).LocY]-[AGENT(iagent).BoxSize]./2;
        Boxes(1,2)  = [AGENT(iagent).LocX]+[AGENT(iagent).BoxSize]./2;
        Boxes(2,2)  = [AGENT(iagent).LocY]+[AGENT(iagent).BoxSize]./2;
        
        pointsidx 	= kdtree_range(tree,Boxes);
        
        % remove the agent itself
        pointsidx(pointsidx==[AGENT(iagent).num]) = [];
        AGENT(iagent).SurroundingAgents = pointsidx;
        
        % Compute the distance to the other agents in the box
        surr_agents = AGENT(iagent).SurroundingAgents;
        x_others    = [AGENT(surr_agents).LocX]';
        y_others    = [AGENT(surr_agents).LocY]';
        others_size = [AGENT(surr_agents).Size]';
        [Normal,Tangent,DistanceToAgents,num_others] = ComputeDistanceToAgents(x_agent,y_agent,agent_size,x_others,y_others,others_size);
        if ~isempty(find(DistanceToAgents==0)); error('fc: dividing by zero!'); end

        if num_others >0
            % find agents that are too close
            indTooClose     = find(DistanceToAgents>0);
            
            %----------------------------------------------------
            % compute social forces from other agents and apply a weighting
            % function to simulate that agents only have a reduced field of
            % vision
            %----------------------------------------------------
            [FxAgentsSocial,FyAgentsSocial,DistanceToAgents] = ComputeSocialForcesDynamic(Parameter,DistanceToAgents,Normal);
            
            %----------------------------------------------------
            % compute physical forces from other agents
            %----------------------------------------------------
            if ~isempty(indTooClose)
                AgentTooClose   = surr_agents(indTooClose);
                DistTooClose    = DistanceToAgents(indTooClose);
                
                velx_others = AGENT(AgentTooClose).VelX;
                vely_others = AGENT(AgentTooClose).VelY;
                
                [FxPhysAgents,FyPhysAgents] = ComputePhysicalForceAgents(velx_agent,vely_agent,velx_others,vely_others,Parameter,DistTooClose,Normal(indTooClose,:),Tangent(indTooClose,:));
                
            else
                FxPhysAgents = 0;
                FyPhysAgents = 0;
            end
        else
            FxAgentsSocial = 0;
            FyAgentsSocial = 0;
            FxPhysAgents = 0;
            FyPhysAgents = 0;
        end
        %----------------------------------------------------
        % compute physical forces from walls 
        %----------------------------------------------------
        [FxPhysWall,FyPhysWall] = ComputePhysicalForceWalls(x_agent,y_agent,agent_size,velx_agent,vely_agent,x_Buildings,y_Buildings,Parameter);

        %----------------------------------------------------
        % assign forces to structure
        %----------------------------------------------------
        AGENT(iagent).FxPhysAgents   = sum(FxPhysAgents);
        AGENT(iagent).FyPhysAgents   = sum(FyPhysAgents);
        AGENT(iagent).FxPhysWall     = sum(FxPhysWall);
        AGENT(iagent).FyPhysWall     = sum(FyPhysWall);
        
        % add some random noise on the social force from other agents
        AGENT(iagent).FxSocialAgents = sum(FxAgentsSocial)*(1+ pert*(-0.5+rand(1)) );
        AGENT(iagent).FySocialAgents = sum(FyAgentsSocial)*(1+ pert*(-0.5+rand(1)) );
    end

    % compute force from exits
    % increase the total social force on the agents by a constant factor
%     if itime > 1
%         soc_force  = sqrt([AGENT.FxSocialWalls]+[AGENT.FxSocialAgents].^2+[AGENT.FySocialWalls]+[AGENT.FySocialAgents].^2);
%     else
%         soc_force = 0;
%     end
% %     
%     xForceExit = [AGENT(1:nagent).xExitDir].*soc_force.*Parameter.ExitFactor;
%     yForceExit = [AGENT(1:nagent).yExitDir].*soc_force.*Parameter.ExitFactor;
%     
    xForceExit = [AGENT(1:nagent).xExitDir].*Parameter.ExitFactor;
    yForceExit = [AGENT(1:nagent).yExitDir].*Parameter.ExitFactor;
    
    xForceExit(xForceExit==0) = [AGENT(1:nagent).xExitDir].*1e5;
    yForceExit(yForceExit==0) = [AGENT(1:nagent).yExitDir].*1e5;
   
    %----------------------------------------------------
    % move agents
    %----------------------------------------------------
    
    % according to [Helbing 2000]:
    % a = dvi/dt = (v0_x - [AGENT.VelX])./t_acc + [AGENT.FxArch]./m + [AGENT.FxPedestrians]./m

    % compute maximum desired velocity due to topography
    % interpolate topography gradient to agents
    agent_gx = interp2(X_Grid,Y_Grid,Gradient_x,[AGENT.LocX],[AGENT.LocY],'*linear');
    agent_gy = interp2(X_Grid,Y_Grid,Gradient_y,[AGENT.LocX],[AGENT.LocY],'*linear');
    
    % compute slope in walking direction
    slope = sum([agent_gx' agent_gy'].*[[AGENT(:).DirX]' [AGENT(:).DirY]'],2);
    % limit maxmimum velocity
    PreFac = ([AGENT(1:nagent).VMax]'./exp(-3.5*0.05));
    V_max_agent = PreFac.*exp(-3.5.*abs(slope+0.05));
    
    
    
    
    % velocity change in x-direction
    v0_x                    = V_max_agent .* [AGENT(1:nagent).xExitDir]';
    dvi_x                   = ( (v0_x - [AGENT.VelX]')./t_acc + [AGENT.FxSocialWalls]'./m + [AGENT.FxSocialAgents]'./m + [AGENT.FxPhysAgents]'./m + [AGENT.FxPhysWall]'./m + xForceExit'./m) .*dt;	%change of velocity
    v_x                     = v0_x + dvi_x;
    
    % velocity change in y-direction
    v0_y                    = V_max_agent .* [AGENT(1:nagent).yExitDir]';
    dvi_y                   = ( (v0_y - [AGENT.VelY]')./t_acc + [AGENT.FySocialWalls]'./m + [AGENT.FySocialAgents]'./m + [AGENT.FyPhysAgents]'./m + [AGENT.FyPhysWall]'./m + yForceExit'./m) .*dt;	%change of velocity
    v_y                     = v0_y + dvi_y;
    
    % compute total velocity and direction of agent
    v_tot                   = sqrt(v_x.^2+v_y.^2);
    dir_x                   = v_x./v_tot;
    dir_y                   = v_y./v_tot;
    
    % recompute maximal velocity due to topgraphy
    % compute slope in walking direction
    slope = sum([agent_gx' agent_gy'].*[dir_x dir_y],2);
    % limit maxmimum velocity
    PreFac = ([AGENT(1:nagent).VMax]'./exp(-3.5*0.05));
    V_max_agent = PreFac.*exp(-3.5.*abs(slope+0.05));
    
    
    % limit velocity to maximum velocity
    v_tot                   = [min(v_tot,V_max_agent)];
    dummy                   = num2cell(v_tot);
    [AGENT(1:nagent).Vel]   = dummy{:};  
    
    % recompute vx and vz
    dummy                   = num2cell(v_tot.*dir_x);
    [AGENT(1:nagent).VelX]  = dummy{:};
    dummy                   = num2cell(v_tot.*dir_y);
    [AGENT(1:nagent).VelY]  = dummy{:};
    
    % update locations
    dummy                   = num2cell([AGENT.LocX] + [AGENT.VelX].*dt);
    [AGENT(1:nagent).LocX]  = dummy{:};                                                     %update x-position
    
    dummy                   = num2cell([AGENT.LocY] + [AGENT.VelY].*dt);
    [AGENT(1:nagent).LocY]  = dummy{:};                                                     %update y-position
    
    % update direction
    dummy = num2cell(dir_x);
    [AGENT(1:nagent).DirX]  = dummy{:};
    dummy = num2cell(dir_y);
    [AGENT(1:nagent).DirY]  = dummy{:};
    
    
    %----------------------------------------------------
    % remove successfull agents
    %----------------------------------------------------
    
    LocX = [AGENT.LocX]; LocY = [AGENT.LocY];
    
    %those who arrived in the exits
    jj=0;
    for i=1%1:size(ExitList,1)
        AGENT(   [AGENT.LocX]>=ExitList(i,1) & [AGENT.LocX]<=ExitList(i,2) ...
            & [AGENT.LocY]>=ExitList(i,3) & [AGENT.LocY]<=ExitList(i,4)  ) = [];
    end
    %remove agents outside model domain
    AGENT(   [AGENT.LocX]>xmax | [AGENT.LocX]<xmin ...
        | [AGENT.LocY]>=ymax | [AGENT.LocY]<ymin  ) = [];
    
    nagent = size(AGENT,2); %update number of agents after removing some of them
    cell_array = num2cell(1:nagent); [AGENT(1:nagent).num] = cell_array{:}; %update correct numbering from 1:nagent

    
    %----------------------------------------------------
    % plot
    %----------------------------------------------------
    
    if mod(itime,5)==0
%     
        % PlotAgents3D(X_Grid,Y_Grid,Z_Grid,AGENT,BuildingList);
        % filename = ['Escape',num2str(itime,'%5.6d')];
        %print(filename,'-djpeg90','-r150')
        
        figure(1),clf
        hold on
        %scatter(X_Grid(:),Y_Grid(:),50,BuildingMap(:),'.')
        % plot topo
%        pcolor(X_Grid,Y_Grid,Z_Grid),shading interp, colorbar
%         caxis([0 0.5])
%         colormap('Bone')
%         colormap(flipud(colormap))
        
        % plot buildings
        PlotBuildings(BuildingList,'r');
        PlotBuildings(ExitList,'g');
        % plot agents
        PlotAgents(nagent,AGENT,'y');
        
        % plot roads
%         for i = 1:size(PathVec,1),plot([X(PathVec(i,1)) X(PathVec(i,2))],[Y(PathVec(i,1)) Y(PathVec(i,2))],'r-'),end
        axis equal
        axis([min(X_Grid(:)) max(X_Grid(:)) min(Y_Grid(:)) max(Y_Grid(:))])
        box on
        title(['time = ',num2str(time/60,3),' min'])
        xlabel('x [m]')
        ylabel('y [m]')
        
    end
end


%==========================================================================