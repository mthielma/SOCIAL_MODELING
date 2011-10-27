%function [STATISTICS, AGENTS] = FloodEvacuation(INPUT,AGENTS,BUILDINGS,STREETS,FLOOD,GRID,TOPOGRAPHY)
clear;

%%%% FABIO SUPERSTAR !!!!!!!!!!!!!!
Debug = 0;
RiseVelocity = 0.001; %water rising velocity in m/s
dt           = 1; %time step in s
nt           = 3600*6; % number of timesteps
nagent       = 100; % number of agents

addpath ./DecisionStrategy/
addpath ./WallForces/
addpath ./Plotting/
%==========================================================================
% initialize fine grid (if not given as argument)

xvec = 0:100;
yvec = 0:100;
[X_Grid,Y_Grid] = meshgrid(xvec,yvec);
Z_Grid = 0*X_Grid; % no topography

% initialize coarse grid for road network
xRoad = 0:5:100;
yRoad = 0:5:100;
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
BuildingList = [45 50 40 50;...
                20 40 12 30;...
                10 16 70 80;...
                70 95 60 80;...
                70 80 50 60;...
                60 85 10 40;...
                45 55 20 35]; % coordinates of building xmin xmax ymin ymax

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
[AGENT(1:nagent).VzMax]     = deal(5);
[AGENT(1:nagent).BoxSize]   = deal(8);

cell_array = num2cell((rand(nagent,1)+1)/2);
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

for iagent = 1:nagent
   AGENT = DecideOnPath(AGENT,PathVec,GradientFactor,Distance); 
end


% plot setup
figure(1),clf
hold on
%scatter(X_Grid(:),Y_Grid(:),50,BuildingMap(:),'.')
axis([min(X_Grid(:)) max(X_Grid(:)) min(Y_Grid(:)) max(Y_Grid(:))])

% plot buildings
PlotBuildings(BuildingList,'r');
% plot agents
PlotAgents(nagent,AGENT,'y');

for i = 1:size(PathVec,1),plot([X(PathVec(i,1)) X(PathVec(i,2))],[Y(PathVec(i,1)) Y(PathVec(i,2))],'k-'),end
axis equal, axis tight

%==========================================================================
% time loop
time = 0;
for itime = 1:nt
    
    %----------------------------------------------------
    % compute water rise
    %----------------------------------------------------
    WaterHeight = WaterHeight + RiseVelocity*dt;
    % compute water level above ground
    WaterLevel  = WaterHeight - Z_grid;
    WaterLevel(WaterLevel<0) = 0;
    
    % compute critical water level
    CriticalWaterLevel = WaterLevel - 0.5;
    
    %----------------------------------------------------
    % get the roads/nodes that are being flooded
    %----------------------------------------------------
    % delete paths at locations where water is covering the roads more than
    % the critical water level
    [PathVec,Distance,Gradient,GradientFactor] = RemovePaths(X_Grid,Y_Grid,CriticalWaterLevel,X,Y,PathVec,Distance,Gradient,GradientFactor);
    
    %----------------------------------------------------
    % compute kdtree of agents for later use
    %----------------------------------------------------
    ReferencePoints(:,1)    = [AGENTS(:).LocX];
    ReferencePoints(:,2)    = [AGENTS(:).LocY];
    
    % generate tree
    tree =   kdtree(ReferencePoints);
    
    % generate the Boxes per Agent
    BoxXmin      = [AGENTS(:).LocX]-[AGENTS(:).BoxSize]./2;
    BoxXmax      = [AGENTS(:).LocX]+[AGENTS(:).BoxSize]./2;
    BoxYmin      = [AGENTS(:).LocY]-[AGENTS(:).BoxSize]./2;
    BoxYmax      = [AGENTS(:).LocY]+[AGENTS(:).BoxSize]./2;
    
    Boxes        = zeros(nagents,2,2);
    Boxes(:,1,1) = BoxXmin;       
    Boxes(:,2,1) = BoxYmin;
    Boxes(:,1,2) = BoxXmax;       
    Boxes(:,2,2) = BoxYmax;
    
    pointsidx    = kdtree_range(tree,Boxes);
    
    % get the agents indices of agents surrounding each agent (individual box)
    
    
    
    %----------------------------------------------------
    % compute forces from flood (on the same grid as the building forces are
    % computed) on all agents
    %----------------------------------------------------
    
    
    %----------------------------------------------------
    % compute forces from buildings on all agents (just interpolate the
    % precomputed force field to the agents
    %----------------------------------------------------
    
    
    
    % agent loop
    for iagent = 1:nagent
        
        %----------------------------------------------------
        % get the agents in a certain range using the kdtree
        %
        % Alternative: use agents that are on the same road
        % since we track this anyway, we do not need kdtree
        %----------------------------------------------------
        
        
        
        %----------------------------------------------------
        % compute social forces from other agents and apply a weighting
        % function to simulate that agents only have a reduced field of
        % vision
        %----------------------------------------------------
        
        %----------------------------------------------------
        % compute physical forces from walls and other agents
        %----------------------------------------------------
        
        %----------------------------------------------------
        % compute force from destination point
        %----------------------------------------------------
        
        
        %----------------------------------------------------
        % compute decisions of agents
        % - in case of extreme slowdown
        % - the agent has reached the destination point
        % - the destination point or a part of the path are flooded
        %----------------------------------------------------
        
        if Decision
            
            switch DecisionCause
                case 1 % slowdown -> turn around and take another path or keep going?
                    AGENT(iagent) = DecideOnPath(AGENT(iagent),PathVec,GradientFactor,Distance,X,Y,Z);
                    
                case 2 % reached destination point -> look in adjacent paths and decide on new exit stategy
                    AGENT(iagent) = DecideOnPathOnCrossing(AGENT(iagent),PathVec,GradientFactor,Distance,X,Y,Z);
                    
                case 3 % destination point is flooded -> node and adjacent paths have been removed from PathVec
                       %                              -> turn around to last node (if this node is also flooded, the agent is deactivated)
                    if ~isempty(find(PathVec==AGENT(iagent).LastDestinationPoint)) % last destination point still exists
                        AGENT(iagent).DestinationPoint =  AGENT(iagent).LastDestinationPoint; % go back
                    else
                        AGENT(iagent).Trapped = true; % agent is trapped -> game over
                    end
            end
            
        end
        
        if AGENT(iagent).Trapped
            % set agent velocity to 0
            
            
        else
            %----------------------------------------------------
            % compute velocity of agents
            % add up forces and compute vx and vy velocity
            %----------------------------------------------------
            
            % limit velocity to max velocity if it is bigger
        end
    end
    
    
    %----------------------------------------------------
    % move agents
    %----------------------------------------------------
    
    
end
%==========================================================================