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
dt                	= 0.05;    	% time step in [s]
maxtime          	= 2000;       % maximum time to run in [min]

%physical parameter
nagent          	= 50;      % number of agents

SocialForce        	= logical(1);   %switch for social force

% physical forces parameters (Helbing,2000)
Parameter.k         = 1.2e5;
Parameter.kappa  	= 2.4e5;

% social force parameters
Parameter.A       	= 2e3;   	%[N]  [2e3 Helbing 2000]
Parameter.B       	= 0.08;      %[m]  [0.08 Helbing 2000]
Parameter.ExitFactor= 1.0;   %for adjusting strength of constant exit force field [30]

% agent parameters
m                 	= 80;       % mass in [kg]
r_agent             = 0.3;      % radius im [m]
v0               	= 0.5;        % maximal/desired velocity [m/s]
cutoffVelocity   	= logical(1); %sets maximum velocity at v0
t_acc            	= 0.5;      % acceleration time in [s]


addpath ./DecisionStrategy/
addpath ./WallForces/
addpath ./Plotting/
addpath ./kdtree_alg_OSX/
%==========================================================================
% initialize fine grid (if not given as argument)
xmin                = 0;
xmax                = 25;
ymin                = 0;
ymax                = 10;

xvec                = xmin:resolution:xmax;
yvec                = ymin:resolution:ymax;
[X_Grid,Y_Grid]     = meshgrid(xvec,yvec);

% set topography
Z_Grid = 0.00.*X_Grid;

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
BoundaryMap(1:3,:)=1; BoundaryMap(size(yvec,2),:)=1; BoundaryMap(:,1)=1; BoundaryMap(:,size(xvec,2))=1;

%---------------------------------------
% create building list (if not given)
%---------------------------------------
BuildingList = [
                0 1  0 10   %boundary wall
                0 25 0 1    %boundary wall
                0 25 9 10   %boundary wall
                15 16 1 4.5   %top barriere
                15 16 5.5 9   %bottom barriere
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
            24 25 4 6
           ]; % coordinates of exits: xmin xmax ymin ymax

ExitList(find(ExitList(:,1)>=xmax),:)   = []; %if exit fully outside domain: remove it!
ExitList(find(ExitList(:,3)>=ymax),:)   = []; %if exit fully outside domain: remove it!
ExitList(find(ExitList(:,2)>xmax),2)    = xmax; %adjust exit to domain boundary
ExitList(find(ExitList(:,4)>ymax),2)    = ymax; %adjust exit to domain boundary

ExitMap = X_Grid*0;
% add exits to map
for i=1:size(ExitList,1)
    ExitMap(X_Grid>=ExitList(i,1) & X_Grid<=ExitList(i,2) & Y_Grid>=ExitList(i,3) & Y_Grid<=ExitList(i,4)) = 1;
end

%----------------------------------------------------
% compute forces from buildings (static)
%----------------------------------------------------
xArchForces = zeros(size(xvec,2),size(yvec,2));  %initialise force field x-direction (nx*ny)
yArchForces = zeros(size(xvec,2),size(yvec,2));  %initialise force field y-direction (nx*ny)

ArchGeometry	= BuildingMap;
ARCH.format     = 'map';                    %'list' or 'map'
ARCH.type       = 1;                        %1: repulsive / 2: attractive
Spreading       = {'exp' 'linear' 'const'};
ARCH.spreading	= Spreading{1};
ARCH.force      = 1.0;                      %1 is the same as wall force

[F_arch_walls, xArchForces_walls, yArchForces_walls, xArchDir_walls, yArchDir_walls] = f_RepWalls3 (X_Grid, Y_Grid, ArchGeometry, ARCH, Parameter);

%add contribution of object(s)
% xArchForces = xArchForces + xArchForces_walls;
% yArchForces = yArchForces + yArchForces_walls;
% xArchForces = xArchForces + xArchForces_walls;
% yArchForces = yArchForces + yArchForces_walls;
% 
% 
% checkFigure = logical(1);
% if checkFigure
%     figure(1),clf
%     quiver(X_Grid',Y_Grid',xArchForces,yArchForces)
%     title('architecture force')
%     xlabel('x [m]')
%     ylabel('y [m]')
%     axis equal; axis tight 
% end

%----------------------------------------------------
% compute forces from exits (static)
%----------------------------------------------------
ArchGeometry	= ExitList;
ARCH.format     = 'list';                   %'list' or 'map'
ARCH.type    	= 2;                        %1: repulsive / 2: attractive
Spreading       = {'exp' 'linear' 'const'};
ARCH.spreading  = Spreading{3};
ARCH.force      = 0.2;                      %1 is the same as wall force

[F_arch_exit, xArchForces_exits, yArchForces_exits, xArchDir_exits, yArchDir_exits] = f_RepWalls3 (X_Grid, Y_Grid, ArchGeometry, ARCH, Parameter);

%add contribution of object(s)
% xArchForces = xArchForces + xArchForces_exits;
% yArchForces = yArchForces + yArchForces_exits;
% 
% checkFigure = logical(1);
% if checkFigure
%     figure(1),clf
%     quiver(X_Grid',Y_Grid',xArchForces,yArchForces)
%     title('architecture force')
%     xlabel('x [m]')
%     ylabel('y [m]')
%     axis equal; axis tight 
% end


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
[AGENT(1:nagent).BoxSize]   = deal(10); % the agent is only influenced by 
[AGENT(1:nagent).Vel]       = deal(0); % initial velocity (to be randomized)

[AGENT(1:nagent).VelX]      = deal(0); % initial velocity X
[AGENT(1:nagent).VelY]      = deal(0); % initial velocity Y

[AGENT(1:nagent).DirX]      = deal(1./sqrt(2)); % x-direction vector
[AGENT(1:nagent).DirY]      = deal(1./sqrt(2)); % y-direction vector

[AGENT(1:nagent).FxSoc]  	= deal(0); % initial social force X
[AGENT(1:nagent).FySoc] 	= deal(0); % initial social force Y
[AGENT(1:nagent).FxArch] 	= deal(0); % initial architecture force X
[AGENT(1:nagent).FyArch]  	= deal(0); % initial architecture force Y
[AGENT(1:nagent).FWalls] 	= deal(0); % initial wall force
[AGENT(1:nagent).FxExits] 	= deal(0); % initial architecture force X
[AGENT(1:nagent).FyExits]  	= deal(0); % initial architecture force Y

[AGENT(1:nagent).DirXwalls] = deal(0); % initial architecture force X
[AGENT(1:nagent).DirYwalls] = deal(0); % initial architecture force Y

[AGENT(1:nagent).DirXexit] 	= deal(0); % initial architecture force X
[AGENT(1:nagent).DirYexit]  = deal(0); % initial architecture force Y

VelX                        = num2cell([AGENT(1:nagent).Vel].*[AGENT(1:nagent).DirX]); % initial velocity X
VelY                        = num2cell([AGENT(1:nagent).Vel].*[AGENT(1:nagent).DirY]); % initial velocity Y


[AGENT(1:nagent).VelX]      = VelX{:};
[AGENT(1:nagent).VelY]      = VelY{:};

%random size
% cell_array = num2cell((0.5+ (rand(nagent,1))*0.2)./2);
% [AGENT(1:nagent).Size]   = cell_array{:};
%same size
[AGENT(1:nagent).Size]   = deal(r_agent);  %radius [m]




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

% plot setup
figure(1),clf
hold on
% plot buildings
PlotBuildings(BuildingList,'r');
PlotBuildings(ExitList,'g');
% plot agents
PlotAgents2(nagent,AGENT,'y');
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
    % building locations
    %----------------------------------------------------
    x_Buildings = X_Grid(BuildingMap==1);
    y_Buildings = Y_Grid(BuildingMap==1);

    %----------------------------------------------------
    % compute forces from buildings on all agents (just interpolate the
    % precomputed force field to the agents)
    %----------------------------------------------------
if isnan(xArchForces(find(isnan(xArchForces)))); error('fc: NaN!'); end
    
    FxArchAgents = interp2(X_Grid,Y_Grid,xArchForces',[AGENT.LocX],[AGENT.LocY],'*linear');
    FyArchAgents = interp2(X_Grid,Y_Grid,yArchForces',[AGENT.LocX],[AGENT.LocY],'*linear');
    
    dummy = num2cell(FxArchAgents);
    [AGENT(1:nagent).FxArch]       = dummy{:};
    dummy = num2cell(FyArchAgents);
    [AGENT(1:nagent).FyArch]       = dummy{:};
    
if isnan([AGENT(find(isnan([AGENT.FxArch]))).FxArch]); error('fc: NaN!'); end

    %walls
    FwallsAgents = interp2(X_Grid,Y_Grid,F_arch_walls',[AGENT.LocX],[AGENT.LocY],'*linear');
    
    dummy = num2cell(FwallsAgents);
    [AGENT(1:nagent).FWalls]       = dummy{:};

    %exit
    FxExitAgents = interp2(X_Grid,Y_Grid,xArchForces_exits',[AGENT.LocX],[AGENT.LocY],'*linear');
    FyExitAgents = interp2(X_Grid,Y_Grid,yArchForces_exits',[AGENT.LocX],[AGENT.LocY],'*linear');
    
    dummy = num2cell(FxExitAgents);
    [AGENT(1:nagent).FxExits]       = dummy{:};
    dummy = num2cell(FyExitAgents);
    [AGENT(1:nagent).FyExits]       = dummy{:};

    %----------------------------------------------------
    % compute direction field to walls on all agents 
    % (just interpolate the precomputed field to the agents)
    %----------------------------------------------------
    xWallsDirAgents = interp2(X_Grid,Y_Grid,xArchDir_walls',[AGENT.LocX],[AGENT.LocY],'*linear');
    yWallsDirAgents = interp2(X_Grid,Y_Grid,yArchDir_walls',[AGENT.LocX],[AGENT.LocY],'*linear');
    
    dummy = num2cell(xWallsDirAgents);
    [AGENT(1:nagent).DirXwalls]       = dummy{:};
    dummy = num2cell(yWallsDirAgents);
    [AGENT(1:nagent).DirYwalls]       = dummy{:};

    
    %----------------------------------------------------
    % compute direction field to exits on all agents 
    % (just interpolate the precomputed field to the agents)
    %----------------------------------------------------
    
    xExitDirAgents = interp2(X_Grid,Y_Grid,xArchDir_exits',[AGENT.LocX],[AGENT.LocY],'*linear');
    yExitDirAgents = interp2(X_Grid,Y_Grid,yArchDir_exits',[AGENT.LocX],[AGENT.LocY],'*linear');
    
    dummy = num2cell(xExitDirAgents);
    [AGENT(1:nagent).DirXexit]       = dummy{:};
    dummy = num2cell(yExitDirAgents);
    [AGENT(1:nagent).DirYexit]       = dummy{:};
    
if isnan(yExitDirAgents(find(isnan(yExitDirAgents)))); error('fc: isnan!'); end


    %----------------------------------------------------
    % agent loop
    iagent=0; nagent2 = nagent;
    for iagent2 = 1:nagent2
        iagent = iagent+1;
        
        % set variable to default value
        TooClose = false;
        
        x_agent = AGENT(iagent).LocX;
        y_agent = AGENT(iagent).LocY;
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
        x_others    = [AGENT([AGENT(iagent).SurroundingAgents]).LocX]';
        y_others    = [AGENT([AGENT(iagent).SurroundingAgents]).LocY]';
        % DistanceToAgents = sqrt((x_agent - x_others).^2+(y_agent - y_others).^2)-([AGENT(iagent).Size]+[AGENT(pointsidx).Size]');
        
        DistanceToAgents    = sqrt((x_agent-x_others).^2 + (y_agent-y_others).^2);  %distance between center of mass
        %check if correct !!!!
        DistanceToAgents_r  = ([AGENT(iagent).Size]+[AGENT(pointsidx).Size]' - DistanceToAgents);  %distance between boundaries ( r_ij - d_ij )
        %check if correct !!!!
if ~isempty(find(DistanceToAgents==0)); error('fc: dividing by zero!'); end


        % compute normal vector        
        NormalX         = (x_agent - x_others)./DistanceToAgents;  %DistanceToAgents should not be zero!
        NormalY         = (y_agent - y_others)./DistanceToAgents;
        
        % find agents that are too close
        indTooClose     = find(DistanceToAgents_r>=0);
        
        %----------------------------------------------------
        % compute social forces from other agents and apply a weighting
        % function to simulate that agents only have a reduced field of
        % vision
        %----------------------------------------------------
        F_socAgents     = Parameter.A.*exp(DistanceToAgents_r./Parameter.B);
        F_socAgentsX    = F_socAgents.*NormalX;
        F_socAgentsY    = F_socAgents.*NormalY;

        
        %----------------------------------------------------
        % compute physical forces from other agents
        %----------------------------------------------------
        if ~isempty(indTooClose)
            % normal force
            F_physAgents_normalX = Parameter.k.*DistanceToAgents_r(indTooClose).*NormalX(indTooClose);
            F_physAgents_normalY = Parameter.k.*DistanceToAgents_r(indTooClose).*NormalY(indTooClose);
            % tangential force
            % compute tangential vector
            TangentX 	= -NormalY(indTooClose);
            TangentY	= NormalX(indTooClose);
            
%             VelOthers   = [AGENT(pointsidx(indTooClose)).Vel];
%             DirXOthers  = [AGENT(pointsidx(indTooClose)).DirX];
%             DirYOthers  = [AGENT(pointsidx(indTooClose)).DirY];
            
%             DeltaVt      = (VelOthers.*DirXOthers-[AGENT(iagent).Vel].*[AGENT(iagent).DirX]).*TangentX' ...
%                         + (VelOthers.*DirYOthers-[AGENT(iagent).Vel].*[AGENT(iagent).DirY]).*TangentY';    
            DeltaVt      = ([AGENT(indTooClose).VelX]-[AGENT(iagent).VelX]).*TangentX' ...
                        + ([AGENT(indTooClose).VelY]-[AGENT(iagent).VelY]).*TangentY';
%             DeltaVt      = ([AGENT(indTooClose).Vel]-[AGENT(iagent).Vel]).*TangentX' ...
%                         + ([AGENT(indTooClose).Vel]-[AGENT(iagent).Vel]).*TangentY';
                    
            F_physAgents_tangentX = Parameter.kappa.*DistanceToAgents_r(indTooClose).*DeltaVt'.*TangentX;
            F_physAgents_tangentY = Parameter.kappa.*DistanceToAgents_r(indTooClose).*DeltaVt'.*TangentY;
        else
            F_physAgents_normalX = 0;
            F_physAgents_normalY = 0;
            F_physAgents_tangentX = 0;
            F_physAgents_tangentY = 0;
        end
        
%         F_physAgents_normalX = 0;
%         F_physAgents_normalY = 0;
%         F_physAgents_tangentX = 0;
%         F_physAgents_tangentY = 0;
        
        
        
        
if isnan([AGENT(find(isnan([AGENT.FxSoc]))).FxSoc]); error('fc: NaN!'); end
if isnan([AGENT(find(isnan([AGENT.FySoc]))).FySoc]); error('fc: NaN!'); end
        
        %----------------------------------------------------
        % add social and physical forces to agent
        %----------------------------------------------------
        dummy = num2cell( sum(F_socAgentsX) + sum(F_physAgents_normalX) + sum(F_physAgents_tangentX) );  %add all social x-forces
        [AGENT(iagent).FxSoc] = dummy{:};
        dummy = num2cell( sum(F_socAgentsY) + sum(F_physAgents_normalY) + sum(F_physAgents_tangentY) );  %add all social y-forces
        [AGENT(iagent).FySoc] = dummy{:};
        
if isnan([AGENT(find(isnan([AGENT.FxSoc]))).FxSoc]); error('fc: NaN!'); end
if isnan([AGENT(find(isnan([AGENT.FySoc]))).FySoc]); error('fc: NaN!'); end

        
                
        %-------------------------------------------------
        % get the distance to the closest wall
        %-------------------------------------------------
        WallDist        = sqrt((x_Buildings-x_agent).^2+(y_Buildings-y_agent).^2); 	%between agent's center of mass and wall boundary    %NOT NEEDED TO COMPUTE HERE! ALREADY DONE!
        minWallDist     = min(WallDist);
        WallDist_r      = WallDist-[AGENT(iagent).Size];                            %between agent's boundary and wall boundary
        minWallDist_r   = min(WallDist_r);
        indWallDist     = find(WallDist_r==minWallDist_r);  %adjust to make sure it is only one single value!!!!
        if minWallDist_r<0
            AtWall = true;
        else
            AtWall = false;
        end

        
        %----------------------------------------------------
        % add social wall force:
        %----------------------------------------------------
        % A*exp[(r-d)/B] = A*exp[-d/B] * exp[r/B]    ...from [Helbing 2000]
        % ...and times normal vector (DirXwalls)
        % ...this is adding exp[r/B] :
        dummy = num2cell( ( [AGENT(iagent).FWalls] * exp([AGENT(iagent).Size]/Parameter.B) )*[AGENT(iagent).DirXwalls] );  %wall force in x-direction
        [AGENT(iagent).FxArch] = dummy{:};
        dummy = num2cell( ( [AGENT(iagent).FWalls] * exp([AGENT(iagent).Size]/Parameter.B) )*[AGENT(iagent).DirYwalls] );  %wall force in y-direction
        [AGENT(iagent).FyArch] = dummy{:};
        %----------------------------------------------------
        
        %----------------------------------------------------
        % add social exit force:
        %----------------------------------------------------
        % F_walls + F_exits
        dummy = num2cell( [AGENT(iagent).FxArch] + [AGENT(iagent).FxExits] );  %exit force in x-direction
        [AGENT(iagent).FxArch] = dummy{:};
        dummy = num2cell( [AGENT(iagent).FyArch] + [AGENT(iagent).FyExits] );  %exit force in y-direction
        [AGENT(iagent).FyArch] = dummy{:};
        %----------------------------------------------------

                
        %----------------------------------------------------
        % compute physical forces from walls 
        %----------------------------------------------------
        
        if AtWall
            if minWallDist==0
                error('fc: minWallDist=0!')
            end
            % compute normal vector   %MIGHT ALREADY BE COMPUTED EARLIER ON
            NormalX_wall = (x_agent-x_Buildings(indWallDist))/minWallDist;      %distance between center of mass    %check that minWallDist_r never becomes zero!
            NormalY_wall = (y_agent-y_Buildings(indWallDist))/minWallDist;      %distance between center of mass    %check that minWallDist_r never becomes zero!
            % compute tangential vector
            TangentX_wall= -NormalY_wall;
            TangentY_wall= NormalX_wall;
            % normal force: k*g*(r-d)*n
            % F_walls + F_exits + F_walls_normal
            dummy = num2cell([AGENT(iagent).FxArch] + (Parameter.k*(-minWallDist_r))*NormalX_wall);   % -minWallDist_r because r-d and not d-r
            [AGENT(iagent).FxArch] = dummy{:};
            dummy = num2cell([AGENT(iagent).FyArch] + (Parameter.k*(-minWallDist_r))*NormalY_wall);   % -minWallDist_r because r-d and not d-r
            [AGENT(iagent).FyArch] = dummy{:};
            % tangential force: k*g*(r-d)*(v*t)*t
            % F_walls + F_exits + F_walls_normal + F_walls_tangential
            dummy = num2cell([AGENT(iagent).FxArch] - (Parameter.kappa*(-minWallDist_r)*[AGENT(iagent).VelX]*TangentX_wall )*TangentX_wall);
            [AGENT(iagent).FxArch] = dummy{:};
            dummy = num2cell([AGENT(iagent).FyArch] - (Parameter.kappa*(-minWallDist_r)*[AGENT(iagent).VelY]*TangentY_wall )*TangentY_wall);
            [AGENT(iagent).FyArch] = dummy{:};
        end

        

    end

    if ~SocialForce; [AGENT(1:nagent).FxSoc] = deal(0); [AGENT(1:nagent).FySoc] = deal(0); end  %switch off social forces
   
    %----------------------------------------------------
    % move agents
    %----------------------------------------------------
    
    % according to [Helbing 2000]:
    % a = dvi/dt = (v0_x - [AGENT.VelX])./t_acc + [AGENT.FxArch]./m + [AGENT.FxPedestrians]./m
    
    
    % velocity change in x-direction
    v0_x                    = [AGENT(1:nagent).VMax] .* [AGENT(1:nagent).DirXexit];
    dvi_x                   = ( (v0_x - [AGENT.VelX])./t_acc + [AGENT.FxSoc]./m + [AGENT.FxArch]./m ) .*dt;	%change of velocity
    v_x                     = v0_x + dvi_x;
    
    % velocity change in y-direction
    v0_y                    = [AGENT(1:nagent).VMax] .* [AGENT(1:nagent).DirYexit];
    dvi_y                   = ( (v0_y - [AGENT.VelY])./t_acc + [AGENT.FySoc]./m + [AGENT.FyArch]./m ) .*dt;	%change of velocity
    v_y                     = v0_y + dvi_y;
    
    % compute total velocity direction of agent
    v_tot                   = sqrt(v_x.^2+v_y.^2);
    dir_x                   = v_x./v_tot;
    dir_y                   = v_y./v_tot;
    
    % compute maximal velocity due to topgraphy

    % interpolate topography gradient to agents
    agent_gx = interp2(X_Grid,Y_Grid,Gradient_x,[AGENT.LocX],[AGENT.LocY]);
    agent_gy = interp2(X_Grid,Y_Grid,Gradient_y,[AGENT.LocX],[AGENT.LocY]);
    
    % compute slope in walking direction
    slope = sum([agent_gx' agent_gy'].*[[AGENT(:).DirX]' [AGENT(:).DirY]'],2);
    % limit maxmimum velocity
    PreFac = ([AGENT(1:nagent).VMax]'./exp(-3.5*0.05));
    V_max_agent = PreFac.*exp(-3.5.*abs(slope-0.05));
    
    % limit velocity to maximum velocity
    v_tot                   = [min(v_tot',V_max_agent)]';
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
    
    %----------------------------------------------------
    % remove successfull agents
    %----------------------------------------------------
    
    LocX = [AGENT.LocX]; LocY = [AGENT.LocY];
    
    %those who arrived in the exits
    jj=0;
    for i=1:size(ExitList,1)
        AGENT(   [AGENT.LocX]>=ExitList(i,1) & [AGENT.LocX]<=ExitList(i,2) ...
            & [AGENT.LocY]>=ExitList(i,3) & [AGENT.LocY]<=ExitList(i,4)  ) = [];
    end
    %remove agents outside model domain
    AGENT(   [AGENT.LocX]>xmax & [AGENT.LocX]<xmin ...
        & [AGENT.LocY]>=ymax & [AGENT.LocY]<ymin  ) = [];
    
    nagent = size(AGENT,2); %update number of agents after removing some of them
    cell_array = num2cell(1:nagent); [AGENT(1:nagent).num] = cell_array{:}; %update correct numbering from 1:nagent

    
    %----------------------------------------------------
    % plot
    %----------------------------------------------------
    
    if mod(itime,10)==0
    
        figure(1),clf
        hold on
        %scatter(X_Grid(:),Y_Grid(:),50,BuildingMap(:),'.')
        % plot topo
%         contour(X_Grid,Y_Grid,Z_Grid,'k-'),shading interp, colorbar
%         caxis([0 0.5])
%         colormap('Bone')
%         colormap(flipud(colormap))
        
        % plot buildings
        PlotBuildings(BuildingList,'r');
        PlotBuildings(ExitList,'g');
        % plot agents
        PlotAgents2(nagent,AGENT,'y');
        
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