%function [STATISTICS, AGENTS] = FloodEvacuation(INPUT,AGENTS,BUILDINGS,STREETS,FLOOD,GRID,TOPOGRAPHY)
clear;

Debug           = 0;
RiseVelocity    = 0.01; %water rising velocity in m/s
dt              = 0.1; %time step in s
nt              = 100; % number of timesteps
nagent          = 100; % number of agents
criticalDepth   = 0.5;      %critical water depth

noUSEatPresent = logical(0);

% physical forces parameters (Helbing,2000)
k       = 1.2e5;
kappa   = 2.4e5;

% social force parameters
A = 2e3;
B = 0.08;

% agent parameters
m           = 80; % mass in kg
v           = 1; % maximal velocity
t_acc       = 0.5; % acceleration itme in s


addpath ./DecisionStrategy/
addpath ./WallForces/
addpath ./Plotting/
addpath ./kdtree_alg_OSX/
%==========================================================================
% initialize fine grid (if not given as argument)
xmin = 0;
xmax = 50;
ymin = 0;
ymax = 10;


xvec = xmin:1:xmax;
yvec = ymin:1:ymax;
[X_Grid,Y_Grid] = meshgrid(xvec,yvec);

Z_Grid = -100./([(X_Grid+100)])+10;
Z_add = 0.1.*abs(Y_Grid-85)-0.5*fliplr(X_Grid)./([fliplr(X_Grid)/10]+20);
Z_Grid = Z_Grid+Z_add*0.25;
Z_Grid(Z_Grid<0) = 0;
% 

% set min to 0
Z_Grid = Z_Grid-min(Z_Grid(:));

% scale to max 5 m height
Z_Grid = 3*(Z_Grid./max(Z_Grid(:)));

% initialize coarse grid for road network
xRoad = xmin:5:xmax;
yRoad = ymin:5:ymax;
[XRoad,YRoad] = meshgrid(xRoad,yRoad);
ZRoad = interp2(X_Grid,Y_Grid,Z_Grid,XRoad,YRoad,'linear');



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
            49 50 1 9
           ]; % coordinates of exits: xmin xmax ymin ymax

ExitList(find(ExitList(:,1)>=xmax),:) = []; %if exit fully outside domain: remove it!
ExitList(find(ExitList(:,3)>=ymax),:) = []; %if exit fully outside domain: remove it!
ExitList(find(ExitList(:,2)>xmax),2) = xmax; %adjust exit to domain boundary
ExitList(find(ExitList(:,4)>ymax),2) = ymax; %adjust exit to domain boundary

ExitMap = X_Grid*0;
% add exits to map
for i=1:size(ExitList,1)
    ExitMap(X_Grid>=ExitList(i,1) & X_Grid<=ExitList(i,2) & Y_Grid>=ExitList(i,3) & Y_Grid<=ExitList(i,4)) = 1;
end

%----------------------------------------------------
% compute forces from buildings (static)
%----------------------------------------------------
xArchForces=zeros(size(xvec,2),size(yvec,2));  %initialise force field x-direction (nx*ny)
yArchForces=zeros(size(xvec,2),size(yvec,2));  %initialise force field y-direction (nx*ny)

nx          = size(xvec,2);
ny          = size(yvec,2);
Arch        = BuildingMap;
ArchFormat  = 'map';                    %'list' or 'map'
Type        = 1;                        %1: repulsive / 2: attractive
Spreading   = {'exp' 'linear' 'const'};
Spreading   = Spreading{1};
Force       = 1.0;                      %1 is the same as wall force

[xArchForces_single, yArchForces_single, grid] = f_RepWalls_single (nx, ny, Arch, ArchFormat, Type, Spreading, Force);

% compute the actual force
xArchForces_single = A.*xArchForces_single.^(1./B);
yArchForces_single = A.*yArchForces_single.^(1./B);

%add contribution of object(s)
xArchForces = xArchForces + xArchForces_single;
yArchForces = yArchForces + yArchForces_single;

checkFigure = logical(1);
if checkFigure
    figure(11)
    quiver(grid(:,:,1),grid(:,:,2),xArchForces,yArchForces)
    title('architecture force')
    xlabel('x')
    ylabel('y')
    axis equal; axis tight 
end

%----------------------------------------------------
% compute forces from exits (static)
%----------------------------------------------------
Arch        = ExitList;
ArchFormat  = 'list';                    %'list' or 'map'
Type        = 2;                        %1: repulsive / 2: attractive
Spreading   = {'exp' 'linear' 'const'};
Spreading   = Spreading{3};
Force       = 0.2;                      %1 is the same as wall force

[xArchForces_single, yArchForces_single, grid] = f_RepWalls_single (nx, ny, Arch, ArchFormat, Type, Spreading, Force);

%add contribution of object(s)
xArchForces = xArchForces + xArchForces_single;
yArchForces = yArchForces + yArchForces_single;




checkFigure = logical(1);
if checkFigure
    figure(11)
    quiver(grid(:,:,1),grid(:,:,2),xArchForces,yArchForces)
    title('architecture force')
    xlabel('x')
    ylabel('y')
    axis equal; axis tight 
end



%==========================================================================
% initialize water height to 0
WaterHeight= X_Grid*0;

%==========================================================================
% initialize graph with nodes, paths between them and cost per path
% - cost due to topography
% - cost due to street width
% - cost due to street type (paved, unpaved)
% - add up all costs

[X,Y,Z,PathVec,Distance,Gradient,GradientFactor] = GenerateRectangularGraph(XRoad,YRoad,ZRoad);

% delete paths at locations where there are buildings covering the nodes
[PathVec,Distance,Gradient,GradientFactor] = RemovePaths(X_Grid,Y_Grid,BuildingMap,X,Y,PathVec,Distance,Gradient,GradientFactor);

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
[AGENT(1:nagent).VMax]      = deal(v);
[AGENT(1:nagent).BoxSize]   = deal(10); % the agent is only influenced by 
[AGENT(1:nagent).Vel]       = deal(1); % initial velocity (to be randomized)
[AGENT(1:nagent).DirX]      = deal(1./sqrt(2)); % x-direction vector
[AGENT(1:nagent).DirY]      = deal(1./sqrt(2)); % y-direction vector

cell_array = num2cell((0.5+ (rand(nagent,1))*0.2)./2);
[AGENT(1:nagent).Size]   = cell_array{:};

% get the indices in BuildingMap that correspond to a road
[iy,ix] = find(BuildingMap==0);
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

% find the road each agents is on and create an initial exit strategy

% for iagent = 1:nagent
%     
%     % get the path
%     
%     
%     % is the agent in the proximity of a node?
%     
%     
%     
%     switch DecisionCause
%         case 1 % slowdown -> turn around and take another path or keep going?
%             AGENT(iagent) = DecideOnPath(AGENT(iagent),PathVec,GradientFactor,Distance,X,Y,Z);
%             
%         case 2 % reached destination point -> look in adjacent paths and decide on new exit stategy
%             AGENT(iagent) = DecideOnPathOnCrossing(AGENT(iagent),PathVec,GradientFactor,Distance,X,Y,Z);
%     end
% end


% plot setup
figure(1),clf
hold on
%scatter(X_Grid(:),Y_Grid(:),50,BuildingMap(:),'.')
axis([min(X_Grid(:)) max(X_Grid(:)) min(Y_Grid(:)) max(Y_Grid(:))])
% plot topo
contour(X_Grid,Y_Grid,Z_Grid,'k-'),shading interp, colorbar
% plot buildings
PlotBuildings(BuildingList,'r');
PlotBuildings(ExitList,'g');
% plot agents
PlotAgents(nagent,AGENT,'y');

% plot roads
for i = 1:size(PathVec,1),plot([X(PathVec(i,1)) X(PathVec(i,2))],[Y(PathVec(i,1)) Y(PathVec(i,2))],'r-'),end
axis equal, axis tight

%==========================================================================
% time loop
time = 0;
for itime = 1:nt
    disp('*****************************************')
    disp(['timestep ',num2str(itime)])

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
    
    
    %----------------------------------------------------
    % building locations
    %----------------------------------------------------
    x_Buildings = X_Grid(BuildingMap==1);
    y_Buildings = Y_Grid(BuildingMap==1);

    
    %----------------------------------------------------
    % compute forces from buildings on all agents (just interpolate the
    % precomputed force field to the agents
    %----------------------------------------------------
    
%     FxArchAgents = interp2(X_Grid,Y_Grid,xArchForces',[AGENT(1:nagent).LocX],[AGENT(1:nagent).LocY]);
%     FyArchAgents = interp2(X_Grid,Y_Grid,yArchForces',[AGENT(1:nagent).LocX],[AGENT(1:nagent).LocY]);
%     
%     dummy = num2cell(FxArchAgents);
%     [AGENT(1:nagent).FxArch]       = dummy{:};
%     dummy = num2cell(FyArchAgents);
%     [AGENT(1:nagent).FyArch]       = dummy{:};
    
    %----------------------------------------------------
    % agent loop
    for iagent = 1:nagent
        
        x_agent = AGENT(iagent).LocX;
        y_agent = AGENT(iagent).LocY;
        %-------------------------------------------------
        % compute the destination vector
        %-------------------------------------------------
        
        
        
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
        pointsidx([AGENT.(pointsidx).num]==AGENT(iagent).num) = [];
        AGENT(iagent).SurroundingAgents = pointsidx;
        
        % Compute the distance to the other agents in the box
        x_others = [AGENT(AGENT(iagent).SurroundingAgents).LocX]';
        y_others = [AGENT(AGENT(iagent).SurroundingAgents).LocY]';
        DistanceToAgents = sqrt((x_others-x_agent).^2+(y_others-y_agent).^2)-(AGENT(iagent).Size+[AGENT(pointsidx).Size]');
        
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
        WallDist    = WallDist-AGENT(iagent).Size;
        indWallDist = find(WallDist<0);
        if ~isempty(indWallDist)
            AtWall = true;
        else
             AtWall = false;
        end
        
        %-------------------------------------------------
        % compute force from wall
        %-------------------------------------------------
        F_socWalls = A*exp(AGENT(iagent).Size-min(WallDist(:)))/B;
        
        AGENT(iagent).FxArch = F_socWalls*NormalX;
        AGENT(iagent).FyArch = F_socWalls*NormalY;
        
        %----------------------------------------------------
        % compute social forces from other agents and apply a weighting
        % function to simulate that agents only have a reduced field of
        % vision
        %----------------------------------------------------
        F_socAgents = A*exp(DistanceToAgents)/B;
        F_socAgentsX = F_socAgents.*NormalX;
        F_socAgentsY = F_socAgents.*NormalY;
        
        %----------------------------------------------------
        % compute physical forces from other agents
        %----------------------------------------------------
        
        if TooClose
            % normal force
            F_physAgents_normalX = k*DistanceToAgents(indTooClose).*NormalX(indTooClose);
            F_physAgents_normalY = k*DistanceToAgents(indTooClose).*NormalY(indTooClose);
            
            
            % tangential force
            % compute tangential vector
            TangentX        = -NormalY(indTooClose);
            TangentY        = NormalX(indTooClose);
            
            VelOthers   = [AGENT(pointsidx(indTooClose)).Vel];
            DirXOthers  = [AGENT(pointsidx(indTooClose)).DirX];
            DirYOthers  = [AGENT(pointsidx(indTooClose)).DirY];
            
            DeltaV      = (VelOthers.*DirXOthers-AGENT(iagent).Vel.*AGENT(iagent).DirX).*TangentX + (VelOthers.*DirYOthers-AGENT(iagent).Vel.*AGENT(iagent).DirY).*TangentY;            
            %F_physAgents_tangentX = kappa*DistanceToAgents(indTooClose).*DeltaV.*TangentX;
            %F_physAgents_tangentY = kappa*DistanceToAgents(indTooClose).*DeltaV.*TangentY;
            
            F_physAgents_tangentX = 0;
            F_physAgents_tangentY = 0;
        else
            F_physAgents_normalX = 0;
            F_physAgents_normalY = 0;
            F_physAgents_tangentX = 0;
            F_physAgents_tangentY = 0;
        end
        
        %----------------------------------------------------
        % compute physical forces from walls 
        %----------------------------------------------------

        if AtWall
            
            % compute normal vector
            x_wall = X(indWallDist);
            y_wall = Y(indWallDist);
            
            % compute normal vector
            NormalX  = (x_agent - x_wall)./WallDist(indWallDist);
            NormalY  = (y_agent - y_wall)./WallDist(indWallDist);
            
            % compute tangential vector
            TangentX = -NormalY;
            TangentY = NormalX;
            
            % normal force
            F_physWall_normalX = k*WallDist(indWallDist).*NormalX;
            F_physWall_normalY = k*WallDist(indWallDist).*NormalY;
            
            % tangential force
            DeltaV      = (-AGENT(iagent).Vel.*AGENT(iagent).DirX).*TangentX + (-AGENT(iagent).Vel.*AGENT(iagent).DirY).*TangentY;
            %F_physWall_tangentX = kappa*WallDist(indWallDist).*DeltaV.*TangentX;
            %F_physWall_tangentY = kappa*WallDist(indWallDist).*DeltaV.*TangentY;
            
            F_physWall_tangentX = 0;
            F_physWall_tangentY = 0;
        else
            F_physWall_normalX = 0;
            F_physWall_normalY = 0;
            F_physWall_tangentX = 0;
            F_physWall_tangentY = 0;
        end

        
        %-----------------------------------------
        % sum all forces
        %-----------------------------------------
        AGENT(iagent).Fx = sum(AGENT(iagent).FxArch)+sum(F_socAgentsX)+sum(F_physAgents_normalX)+sum(F_physAgents_tangentX)+sum(F_physWall_normalX)+sum(F_physWall_tangentX);
        AGENT(iagent).Fy = sum(AGENT(iagent).FyArch)+sum(F_socAgentsY)+sum(F_physAgents_normalY)+sum(F_physAgents_tangentY)+sum(F_physWall_normalY)+sum(F_physWall_tangentY);
        
        %-----------------------------------------
        % compute the velocity of the agent
        %-----------------------------------------
        time_factor = 1/dt+1/t_acc;
        Vx = AGENT(iagent).Vel*AGENT(iagent).DirX+AGENT(iagent).Fx/time_factor/m;
        Vy = AGENT(iagent).Vel*AGENT(iagent).DirY+AGENT(iagent).Fy/time_factor/m;
        
        AGENT(iagent).Vel = sqrt(Vx^2+Vy^2);
        
        % compute the resulting movement vector
        AGENT(iagent).DirX = Vx/AGENT(iagent).Vel;
        AGENT(iagent).DirY = Vy/AGENT(iagent).Vel;
        
        % limit to max velocity
        if AGENT(iagent).Vel>AGENT(iagent).VMax
            AGENT(iagent).Vel=AGENT(iagent).VMax;
        end
        
        AGENT(iagent).Vx = AGENT(iagent).Vel*AGENT(iagent).DirX;
        AGENT(iagent).Vy = AGENT(iagent).Vel*AGENT(iagent).DirY;
    end
   
    %----------------------------------------------------
    % move agents
    %----------------------------------------------------
    dummy = num2cell([AGENT.LocX] + [AGENT.Vx]);
    [AGENT(1:nagent).LocX] = dummy{:}; 
    
    dummy = num2cell([AGENT.LocY] + [AGENT.Vy]);
    [AGENT(1:nagent).LocY] = dummy{:};
    

    %----------------------------------------------------
    % plot
    %----------------------------------------------------
    
    if mod(itime,1)==0
    
        figure(1),clf
        hold on
        %scatter(X_Grid(:),Y_Grid(:),50,BuildingMap(:),'.')
        axis([min(X_Grid(:)) max(X_Grid(:)) min(Y_Grid(:)) max(Y_Grid(:))])
        
        % plot buildings
        PlotBuildings(BuildingList,'r');
        PlotBuildings(ExitList,'g');
        % plot agents
        PlotAgents(nagent,AGENT,'y');
        
        % plot roads
%         for i = 1:size(PathVec,1),plot([X(PathVec(i,1)) X(PathVec(i,2))],[Y(PathVec(i,1)) Y(PathVec(i,2))],'r-'),end
        axis equal, axis tight
        
        
    end
    %----------------------------------------------------
    % remove successfull agents
    %----------------------------------------------------
    
    ind = find([AGENT.LocX]>ExitList(1));
    AGENT(ind) = [];
    
    
%     find( [AGENT(find( [AGENT.LocX]>=ExitList(1,1) ) ).LocX]<=ExitList(1,2) )
%     find( [AGENT.LocY]>=ExitList(1,3) )
%     find( [AGENT.LocY]<=ExitList(1,4) );
end
%==========================================================================