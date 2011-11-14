%=================================
% 
%          Escape Panic
% 
%=================================
% Marcel Thielmann & Fabio Crameri

% Comments:
% - B, i.e. spread out of wall forces needs to be larger ...fc
% 

clear;

%numerical parameter
resolution      = 0.2;      % resolution in [m]
dt              = 0.1;    	% time step in [s]
maxtime         = 30;       % maximum time to run in [min]

%physical parameter
nagent          = 30;      % number of agents

noUSEatPresent  = logical(0);

% physical forces parameters (Helbing,2000)
k               = 1.2e5;
kappa           = 2.4e5;

% social force parameters
Parameter.A  	= 2e3;   	%[N]  [2e3 Helbing 2000]
Parameter.B  	= 0.1;      %[m]  [0.08 Helbing 2000]

% agent parameters
m               = 80;       % mass in kg
v0              = 1;        % maximal/desired velocity [m/s]
t_acc           = 0.5;      % acceleration time in [s]


addpath ./DecisionStrategy/
addpath ./WallForces/
addpath ./Plotting/
addpath ./kdtree_alg_OSX/
%==========================================================================
% initialize fine grid (if not given as argument)
xmin            = 0;
xmax            = 50;
ymin            = 0;
ymax            = 10;

xvec            = xmin:resolution:xmax;
yvec            = ymin:resolution:ymax;
[X_Grid,Y_Grid] = meshgrid(xvec,yvec);


%convert time
maxtime = maxtime*60; %[min] => [s]

%==========================================================================
% initialize buildings
% - location
% - number of people inside
% - number of families inside
% - location of the exit

%---------------------------------------
% create building list (if not given)
%---------------------------------------
BuildingList = [
                0 50 0 1    %boundary wall
                0 50 9 10   %boundary wall
                30 32 1 4   %top barriere
                30 32 6 9   %bottom barriere
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
            48 50 4 6
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

[xArchForces_walls, yArchForces_walls, xArchDir_walls, yArchDir_walls] = f_RepWalls_single (X_Grid, Y_Grid, ArchGeometry, ARCH, Parameter);

%add contribution of object(s)
xArchForces = xArchForces + xArchForces_walls;
yArchForces = yArchForces + yArchForces_walls;

checkFigure = logical(1);
if checkFigure
    figure(11)
    quiver(X_Grid',Y_Grid',xArchForces,yArchForces)
    title('architecture force')
    xlabel('x [m]')
    ylabel('y [m]')
    axis equal; axis tight 
end

%----------------------------------------------------
% compute forces from exits (static)
%----------------------------------------------------
ArchGeometry	= ExitList;
ARCH.format     = 'list';                   %'list' or 'map'
ARCH.type    	= 2;                        %1: repulsive / 2: attractive
Spreading       = {'exp' 'linear' 'const'};
ARCH.spreading  = Spreading{3};
ARCH.force      = 0.2;                      %1 is the same as wall force

[xArchForces_exits, yArchForces_exits, xArchDir_exits, yArchDir_exits] = f_RepWalls_single (X_Grid, Y_Grid, ArchGeometry, ARCH, Parameter);

%add contribution of object(s)
xArchForces2 = xArchForces + xArchForces_exits;
yArchForces2 = yArchForces + yArchForces_exits;

checkFigure = logical(1);
if checkFigure
    figure(11)
    quiver(X_Grid',Y_Grid',xArchForces2,yArchForces2)
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
[AGENT(1:nagent).num]       = cell_array{:};
[AGENT(1:nagent).VMax]      = deal(v0);
[AGENT(1:nagent).BoxSize]   = deal(10); % the agent is only influenced by 
[AGENT(1:nagent).Vel]       = deal(1); % initial velocity (to be randomized)

[AGENT(1:nagent).VelX]      = deal(0); % initial velocity X
[AGENT(1:nagent).VelY]      = deal(0); % initial velocity Y

[AGENT(1:nagent).DirX]      = deal(1./sqrt(2)); % x-direction vector
[AGENT(1:nagent).DirY]      = deal(1./sqrt(2)); % y-direction vector

cell_array = num2cell((0.5+ (rand(nagent,1))*0.2)./2);
[AGENT(1:nagent).Size]   = cell_array{:};

% get the indices in BuildingMap that correspond to a road
[iy,ix] = find(BuildingMap==0 & ExitMap==0);
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
    
    % update the number of agents
    nagent = size(AGENT,2);
    
    %----------------------------------------------------
    % compute kdtree of agents for later use
    %----------------------------------------------------
    ReferencePoints         = zeros(nagent,2);
    ReferencePoints(:,1)    = [AGENT(:).LocX];
    ReferencePoints(:,2)    = [AGENT(:).LocY];
    
    % generate tree
    tree =   kdtree(ReferencePoints);
    
    clearvars ReferencePoints
    
    %----------------------------------------------------
    % building locations
    %----------------------------------------------------
    x_Buildings = X_Grid(BuildingMap==1);
    y_Buildings = Y_Grid(BuildingMap==1);

    
    %----------------------------------------------------
    % compute forces from buildings on all agents (just interpolate the
    % precomputed force field to the agents)
    %----------------------------------------------------
    
    FxArchAgents = interp2(X_Grid,Y_Grid,xArchForces',[AGENT(1:nagent).LocX],[AGENT(1:nagent).LocY]);
    FyArchAgents = interp2(X_Grid,Y_Grid,yArchForces',[AGENT(1:nagent).LocX],[AGENT(1:nagent).LocY]);
    
    dummy = num2cell(FxArchAgents);
    [AGENT(1:nagent).FxArch]       = dummy{:};
    dummy = num2cell(FyArchAgents);
    [AGENT(1:nagent).FyArch]       = dummy{:};
    
    %----------------------------------------------------
    % compute direction field to exits on all agents 
    % (just interpolate the precomputed field to the agents)
    %----------------------------------------------------
    
    xExitDirAgents = interp2(X_Grid,Y_Grid,xArchDir_exits',[AGENT(1:nagent).LocX],[AGENT(1:nagent).LocY]);
    yExitDirAgents = interp2(X_Grid,Y_Grid,yArchDir_exits',[AGENT(1:nagent).LocX],[AGENT(1:nagent).LocY]);
    
    dummy = num2cell(xExitDirAgents);
    [AGENT(1:nagent).xExitDir]       = dummy{:};
    dummy = num2cell(yExitDirAgents);
    [AGENT(1:nagent).yExitDir]       = dummy{:};
    
    
    
    %----------------------------------------------------
    % agent loop
    for iagent = 1:nagent
        
        x_agent = AGENT(iagent).LocX;
        y_agent = AGENT(iagent).LocY;
        
        %-------------------------------------------------
        % get the agents that are in the "individual box" and compute the
        % distance to them
        %-------------------------------------------------
        
        % generate the Boxes per Agent
        Boxes      = zeros(2,2);
        Boxes(1,1) = [AGENT(iagent).LocX]-[AGENT(iagent).BoxSize]./2;
        Boxes(2,1) = [AGENT(iagent).LocY]-[AGENT(iagent).BoxSize]./2;
        Boxes(1,2) = [AGENT(iagent).LocX]+[AGENT(iagent).BoxSize]./2;
        Boxes(2,2) = [AGENT(iagent).LocY]+[AGENT(iagent).BoxSize]./2;
        
        pointsidx    = kdtree_range(tree,Boxes);
        % remove the agent itself
        pointsidx(pointsidx==[AGENT(iagent).num]) = [];
        AGENT(iagent).SurroundingAgents = pointsidx;
        
        % Compute the distance to the other agents in the box
        x_others = [AGENT([AGENT(iagent).SurroundingAgents]).LocX]';
        y_others = [AGENT([AGENT(iagent).SurroundingAgents]).LocY]';
        DistanceToAgents = sqrt((x_others-x_agent).^2+(y_others-y_agent).^2)-([AGENT(iagent).Size]+[AGENT(pointsidx).Size]');
        
        % compute normal vector
        VecNormal       = zeros(size(pointsidx,2),2);
        
        NormalX         = (x_agent - x_others)./DistanceToAgents;
        NormalY         = (y_agent - y_others)./DistanceToAgents;
        
        % find agents that are too close
        indTooClose                    = find(DistanceToAgents<=0);
        if ~isempty(indTooClose)
            TooClose = true;
        else
            TooClose = false;
        end
        
        %-------------------------------------------------
        % get the distance to the closest wall
        %-------------------------------------------------
        WallDist    = sqrt((x_Buildings-x_agent).^2+(y_Buildings-y_agent).^2);
        WallDist    = WallDist-[AGENT(iagent).Size];
        indWallDist = find(WallDist<0);
        if ~isempty(indWallDist)
            AtWall = true;
        else
            AtWall = false;
        end
        
        %----------------------------------------------------
        % compute social forces from other agents and apply a weighting
        % function to simulate that agents only have a reduced field of
        % vision
        %----------------------------------------------------
        F_socAgents = Parameter.A.*exp(DistanceToAgents)./Parameter.B;
        F_socAgentsX = F_socAgents.*NormalX;
        F_socAgentsY = F_socAgents.*NormalY;
        
        %----------------------------------------------------
        % compute physical forces from other agents
        %----------------------------------------------------
        if noUSEatPresent
        if TooClose
            % normal force
            F_physAgents_normalX = k.*DistanceToAgents(indTooClose).*NormalX(indTooClose);
            F_physAgents_normalY = k.*DistanceToAgents(indTooClose).*NormalY(indTooClose);
            % tangential force
            % compute tangential vector
            TangentX        = -NormalY(v0);
            TangentY        = NormalX(indTooClose);
            
            VelOthers   = [AGENT(pointsidx(indTooClose)).Vel];
            DirXOthers  = [AGENT(pointsidx(indTooClose)).DirX];
            DirYOthers  = [AGENT(pointsidx(indTooClose)).DirY];
            
            DeltaV      = (VelOthers.*DirXOthers-[AGENT(iagent).Vel].*[AGENT(iagent).DirX]).*TangentX ...
                        + (VelOthers.*DirYOthers-[AGENT(iagent).Vel].*[AGENT(iagent).DirY]).*TangentY;    
                    
            F_physAgents_tangentX = kappa.*DistanceToAgents(indTooClose).*DeltaV.*TangentX;
            F_physAgents_tangentY = kappa.*DistanceToAgents(indTooClose).*DeltaV.*TangentY;
        end
        end
        
        %----------------------------------------------------
        % compute physical forces from walls 
        %----------------------------------------------------
        if noUSEatPresent
        if AtWall
            % compute normal vector
            x_wall      = X(indWallDist);
            y_wall      = Y(indWallDist);
            
            % compute normal vector
            NormalX     = (x_agent - x_wall)./WallDist(indWallDist);
            NormalY     = (y_agent - y_wall)./WallDist(indWallDist);
            
            % compute tangential vector
            TangentX    = -NormalY;
            TangentY    = NormalX;
            
            % normal force
            F_physWall_normalX = k*WallDist(indWallDist).*NormalX;
            F_physWall_normalY = k*WallDist(indWallDist).*NormalY;
            
            % tangential force
            DeltaV      = (-[AGENT(iagent).Vel].*[AGENT(iagent).DirX]).*TangentX ...
                + (-[AGENT(iagent).Vel].*[AGENT(iagent).DirY]).*TangentY;
            F_physAgents_tangentX = kappa.*WallDist(indWallDist).*DeltaV.*TangentX;
            F_physAgents_tangentY = kappa.*WallDist(indWallDist).*DeltaV.*TangentY;
        end
        end

    end
    %----------------------------------------------------
    % wall forces
    %----------------------------------------------------
    
    %     f_iW = -[AGENT.FxArch]+k*g...
    
    % A*exp[(r-d)/B] = A*exp[-d/B] * exp[r/B]
    dummy = num2cell([AGENT.FxArch] * exp([AGENT(iagent).Size]/Parameter.B));               %add agent radii to force field
    [AGENT(1:nagent).FxArch] = dummy{:};
    dummy = num2cell([AGENT.FyArch] * exp([AGENT(iagent).Size]/Parameter.B));               %add agent radii to force field
    [AGENT(1:nagent).FyArch] = dummy{:};
   
    %----------------------------------------------------
    % move agents
    %----------------------------------------------------
    
    % according to [Helbing 2000]:
    % a = dvi/dt = (v0_x - [AGENT.VelX])./t_acc + [AGENT.FxArch]./m + [AGENT.FxPedestrians]./m
    v0_x                    = v0 .* [AGENT(1:nagent).xExitDir];
    dvi_x                   = ( (v0_x - [AGENT.VelX])./t_acc + [AGENT.FxArch]./m ) .*dt;	%change of velocity
    dummy                   = num2cell([AGENT.VelX]+dvi_x);
    [AGENT(1:nagent).VelX]  = dummy{:};                                                     %update velocity
    dummy                   = num2cell([AGENT.LocX] + [AGENT.VelX].*dt);
    [AGENT(1:nagent).LocX]  = dummy{:};                                                     %update position
    
    v0_y                    = v0 .* [AGENT(1:nagent).yExitDir];
    dvi_y                   = ( (v0_y - [AGENT.VelY])./t_acc + [AGENT.FyArch]./m ) .*dt;	%change of velocity
    dummy                   = num2cell([AGENT.VelY]+dvi_y);
    [AGENT(1:nagent).VelY]  = dummy{:};                                                     %update velocity
    dummy                   = num2cell([AGENT.LocY] + [AGENT.VelY].*dt);
    [AGENT(1:nagent).LocY]  = dummy{:};                                                     %update position
        
    

    
    
    %----------------------------------------------------
    % remove successfull agents
    %----------------------------------------------------
    
    %those who arrived in the exits
    for i=1:size(ExitList,1)
        AGENT(   [AGENT.LocX]>=ExitList(i,1) & [AGENT.LocX]<=ExitList(i,2) ...
            & [AGENT.LocY]>=ExitList(i,3) & [AGENT.LocY]<=ExitList(i,4)  ) = [];
        
    end
    
    nagent = size([AGENT.LocX],2); %update number of agents after removing some of them
    
    
    %----------------------------------------------------
    % plot
    %----------------------------------------------------
    
    if mod(itime,1)==0
    
        figure(1),clf
        hold on
        %scatter(X_Grid(:),Y_Grid(:),50,BuildingMap(:),'.')
        % plot topo
%         contour(X_Grid,Y_Grid,Z_Grid,'k-'),shading interp, colorbar
        caxis([0 0.5])
        colormap('Bone')
        colormap(flipud(colormap))
        
        % plot buildings
        PlotBuildings(BuildingList,'r');
        PlotBuildings(ExitList,'g');
        % plot agents
        PlotAgents(nagent,AGENT,'y');
        
        % plot roads
%         for i = 1:size(PathVec,1),plot([X(PathVec(i,1)) X(PathVec(i,2))],[Y(PathVec(i,1)) Y(PathVec(i,2))],'r-'),end
        axis equal
        axis([min(X_Grid(:)) max(X_Grid(:)) min(Y_Grid(:)) max(Y_Grid(:))])
        
        title(['time = ',num2str(time/60,3),' min'])
        xlabel('x [m]')
        ylabel('y [m]')
        
    end
end
%==========================================================================