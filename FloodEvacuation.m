%function [STATISTICS, AGENTS] = FloodEvacuation(INPUT,AGENTS,BUILDINGS,STREETS,FLOOD,GRID,TOPOGRAPHY)
clear;

%%%% FABIO SUPERSTAR !!!!!!!!!!!!!!
Debug = 0;
RiseVelocity = 0.01; %water rising velocity in m/s
dt           = 1; %time step in s
nt           = 100; % number of timesteps
nagent       = 2000; % number of agents


% physical forces parameters (Helbing,2000)
k       = 1.2e5;
kappa   = 2.4e5;

% social force parameters
A = 2e3;
B = 0.08;

% agent parameters
m           = 80; % mass in kg
v           = 5; % maximal velocity
t_acc       = 0.5; % acceleration itme in s


addpath ./DecisionStrategy/
addpath ./WallForces/
addpath ./Plotting/
addpath ./kdtree_alg_OSX/
%==========================================================================
% initialize fine grid (if not given as argument)
xmin = 0;
xmax = 100;
ymin = 0;
ymax = 100;


xvec = xmin:1:xmax;
yvec = ymin:1:ymax;
[X_Grid,Y_Grid] = meshgrid(xvec,yvec);
%Z_Grid = 0*X_Grid; % no topography
%Z_Grid = 0.02.*X_Grid;
%Z_Grid(Z_Grid>10) = 10;
%Z_Grid(Z_Grid<0) = 0;

Z_Grid = -100./([(X_Grid+100)])+10;
Z_add = 0.1.*abs(Y_Grid-85)-0.5*fliplr(X_Grid)./([fliplr(X_Grid)/10]+20);
Z_Grid = Z_Grid+Z_add*0.25;
Z_Grid(Z_Grid<0) = 0;
% 
% Z_add = 0.1.*abs(Y_Grid);
% Z_Grid = Z_Grid+Z_add;
% 
% 
% Z_Grid(Z_Grid>20) = 20;
% 
% 
% 
% 
% 
% set min to 0
Z_Grid = Z_Grid-min(Z_Grid(:));

% scale to max 5 m height
Z_Grid = 3*(Z_Grid./max(Z_Grid(:)));

% add some sinusoidal perturbations
%Z_add = 0.1*sin(0.05*(X_Grid+Y_Grid))+cos(0.01*Y_Grid);
%Z_Grid = Z_Grid-Z_add/min(Z_add(:));

% figure(99),clf
% surf(X_Grid,Y_Grid,Z_Grid),shading interp, colorbar
% hold on
% contour3(X_Grid,Y_Grid,Z_Grid,[0.5 0.5],'k-')

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
BuildingList = [42 53 42 53;...
                22 38 12 33;...
                12 16 72 83;...
                72 95 58 86;...
                72 83 48 58;...
                53 67 62 88;...
                22 48 56 78;...
                62 85  8 43;...
                42 57 23 35;...
                 0 28 92 100;...
                 32 97 92 100;...
                 102 197 92 97]; % coordinates of building xmin xmax ymin ymax

BuildingMap = X_Grid*0;
% add buildings to map
for i = 1:size(BuildingList,1)
    BuildingMap(X_Grid>=BuildingList(i,1) & X_Grid<=BuildingList(i,2) & Y_Grid>=BuildingList(i,3) & Y_Grid<=BuildingList(i,4)) = 1;
end

%----------------------------------------------------
% compute forces from buildings (static)
%----------------------------------------------------


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
[AGENT(1:nagent).VMax]      = deal(5);
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
    %----------------------------------------------------
    % compute water rise
    %----------------------------------------------------
    WaterHeight = WaterHeight + RiseVelocity*dt;
    % compute water level above ground
    WaterLevel  = WaterHeight - Z_Grid;
    WaterLevel(WaterLevel<0) = 0;
    
    % compute critical water level
    CriticalWaterLevel = WaterLevel - 0.5;
    %----------------------------------------------------
    % get the roads/nodes that are being flooded
    %----------------------------------------------------
    % delete paths at locations where water is covering the roads more than
    % the critical water level
    
    if max(CriticalWaterLevel(:))>0
        [PathVec,Distance,Gradient,GradientFactor] = RemovePaths(X_Grid,Y_Grid,CriticalWaterLevel,X,Y,PathVec,Distance,Gradient,GradientFactor);
    end
    %----------------------------------------------------
    % compute kdtree of agents for later use
    %----------------------------------------------------
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
    % compute forces from flood (on the same grid as the building forces are
    % computed) on all agents
    %----------------------------------------------------
    
    
    %----------------------------------------------------
    % compute forces from buildings on all agents (just interpolate the
    % precomputed force field to the agents
    %----------------------------------------------------
    
    
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
        Boxes        = zeros(2,2);
        Boxes(1,1) = [AGENT(iagent).LocX]-[AGENT(iagent).BoxSize]./2;
        Boxes(2,1) = [AGENT(iagent).LocY]-[AGENT(iagent).BoxSize]./2;
        Boxes(1,2) = [AGENT(iagent).LocX]+[AGENT(iagent).BoxSize]./2;
        Boxes(2,2) = [AGENT(iagent).LocY]+[AGENT(iagent).BoxSize]./2;
        
        pointsidx    = kdtree_range(tree,Boxes);
        % remove the agent itself
        pointsidx(pointsidx==AGENT(iagent).num) = [];
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
            F_physAgents_tangentX = kappa*DistanceToAgents(indTooClose).*DeltaV.*TangentX;
            F_physAgents_tangentY = kappa*DistanceToAgents(indTooClose).*DeltaV.*TangentY;
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
            F_physAgents_tangentX = kappa*WallDist(indWallDist).*DeltaV.*TangentX;
            F_physAgents_tangentY = kappa*WallDist(indWallDist).*DeltaV.*TangentY;
        end
        
        %----------------------------------------------------
        % compute force from destination point
        %----------------------------------------------------
        
        x_dest = X(AGENT(iagent).DestinationPoint);
        y_dest = Y(AGENT(iagent).DestinationPoint);
        
        
        
        
        %----------------------------------------------------
        % compute decisions of agents
        % - in case of extreme slowdown
        % - the agent has reached the destination point
        % - the destination point or a part of the path are flooded
        %----------------------------------------------------
%         
%         DistanceDestinationPoint
%         % has the agent reached the destination point?
%         
%         
%         % is the destination point flooded?
%         
%         
%         % see if slowdown causes agent to rethink his exit strategy
%         if AGENT(iagent).Velocity/AGENT(iagent).VMax < AGENT(iagent).MaxSlowdown
%             AGENT(iagent).SlowdownSteps = AGENT(iagent).SlowdownSteps+1;
%             if AGENT(iagent).SlowdownSteps > AGENT(iagent).MaxSlowdownSteps % agent is fed up because of slowdown
%                 Decision        = true;
%                 DecisionCause   = 1;
%             else
%                 Decision = false;
%             end
%         else
%             AGENT(iagent).SlowdownSteps = 0; % reset if the agent can move with a satifying velocity
%             Decision = false;
%         end
%         
%         
%         
%         
%         if Decision
%             
%             switch DecisionCause
%                 case 1 % slowdown -> turn around and take another path or keep going?
%                     AGENT(iagent) = DecideOnPath(AGENT(iagent),PathVec,GradientFactor,Distance,X,Y,Z);
%                     
%                 case 2 % reached destination point -> look in adjacent paths and decide on new exit stategy
%                     AGENT(iagent) = DecideOnPathOnCrossing(AGENT(iagent),PathVec,GradientFactor,Distance,X,Y,Z);
%                     
%                 case 3 % destination point is flooded -> node and adjacent paths have been removed from PathVec
%                        %                              -> turn around to last node (if this node is also flooded, the agent is deactivated)
%                     if ~isempty(find(PathVec==AGENT(iagent).LastDestinationPoint)) % last destination point still exists
%                         AGENT(iagent).DestinationPoint =  AGENT(iagent).LastDestinationPoint; % go back
%                     else
%                         AGENT(iagent).Trapped = true; % agent is trapped -> game over
%                     end
%             end
%             
%         end
%         
%         if AGENT(iagent).Trapped
%             % set agent velocity to 0
%             
%             
%         else
%             %----------------------------------------------------
%             % compute velocity of agents
%             % add up forces and compute vx and vy velocity
%             %----------------------------------------------------
%             
%               
%           
%
%             % limit velocity to max velocity if it is bigger
%         end
    end
%     
%     
%     %----------------------------------------------------
%     % move agents
%     %----------------------------------------------------
    
    %----------------------------------------------------
    % plot
    %----------------------------------------------------
    
    if mod(itime,10)==0
    
        figure(1),clf
        hold on
        %scatter(X_Grid(:),Y_Grid(:),50,BuildingMap(:),'.')
        axis([min(X_Grid(:)) max(X_Grid(:)) min(Y_Grid(:)) max(Y_Grid(:))])
        % plot topo
        contour(X_Grid,Y_Grid,Z_Grid,'k-'),shading interp, colorbar
        h = contourf(X_Grid,Y_Grid,WaterLevel,[0.1 0.5],'EdgeColor','none');
        caxis([0 0.5])
        colormap('Bone')
        colormap(flipud(colormap))
        
        % plot buildings
        PlotBuildings(BuildingList,'r');
        % plot agents
        PlotAgents(nagent,AGENT,'y');
        
        % plot roads
%         for i = 1:size(PathVec,1),plot([X(PathVec(i,1)) X(PathVec(i,2))],[Y(PathVec(i,1)) Y(PathVec(i,2))],'r-'),end
        axis equal, axis tight
        
        title(['max water = ',num2str(max(WaterLevel(:)))])
        
    end
end
%==========================================================================