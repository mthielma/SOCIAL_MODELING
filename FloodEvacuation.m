%function [STATISTICS, AGENTS] = FloodEvacuation(INPUT,AGENTS,BUILDINGS,STREETS,FLOOD,GRID,TOPOGRAPHY)


%%%% FABIO SUPERSTAR !!!!!!!!!!!!!!
Debug = 0;


addpath ./DecisionStrategy/
addpath ./WallForces/
%==========================================================================
% initialize fine grid (if not given as argument)

xvec = 0:100;
yvec = 0:100;
[X_Grid,Y_Grid] = meshgrid(xvec,yvec);
Z_Grid = 0*X_Grid; % no topography

% initialize coarse grid for road network
xRoad = 10:10:90;
yRoad = 10:4:90;
[XRoad,YRoad] = meshgrid(xRoad,yRoad);
ZRoad = interp2(X_Grid,Y_Grid,Z_Grid,XRoad,YRoad,'linear');



%==========================================================================
% initialize buildings
% - location
% - number of people inside
% - number of families inside
% - location of the exit

%---------------------------------------
% test: create one building in the middle
%---------------------------------------
BuildingList = [42 49 42 49;...
                20 40 12 30]; % coordinates of building xmin xmax ymin ymax
BuildingMap = X_Grid*0;
for i = 1:size(BuildingList,1)
    BuildingMap(X_Grid>=BuildingList(i,1) & X_Grid<=BuildingList(i,2) & Y_Grid>=BuildingList(i,3) & Y_Grid<=BuildingList(i,4)) = 1;
end

%----------------------------------------------------
% compute forces from buildings (static)
%----------------------------------------------------


%==========================================================================
% initialize graph with nodes, paths between them and cost per path
% - cost due to topography
% - cost due to street width
% - cost due to street type (paved, unpaved)
% - add up all costs

[X,Y,PathVec,Distance,Gradient] = GenerateRectangularGraph(XRoad,YRoad,ZRoad);

% delete paths at locations where there are buildings covering the nodes
[PathVec,Distance,Gradient] = RemovePaths(X_Grid,Y_Grid,BuildingMap,X,Y,PathVec,Distance,Gradient);

%==========================================================================
% initialize agents in buildings and on streets
% - location
% - mass
% - age -> maximum velocity, max acceleration
% - panic factor (how often are new strategies taken, how much are persons in the way influencing the cost of that path?)
% - horizon distance (how far can the agent look?)
% - group (e.g. family) --> later
% - helping factor --> later


%==========================================================================
% time loop
for itime = 1:nt
    
    %----------------------------------------------------
    % compute water rise
    %----------------------------------------------------
    
    %----------------------------------------------------
    % get the roads/nodes that are being flooded
    %----------------------------------------------------
    
    %----------------------------------------------------
    % compute kdtree of agents for later use
    %----------------------------------------------------
    
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
        % since we track this, we do not need kdtree
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
           % add the paths between the agent and the next crossings (nodes)
           % to the path vector
           maxNode   = max(PathVec(:));
           NodeAgent = maxNode+1;
           
           [NodeAgent NodeEnd]
           [NodeAgent NodeStart]
           
           % compute the additional cost due to people in the way
           
           
           
           % Find the shortest path via a MATLAB function
           % Using the Bioinformatics Toolbox? function graphshortestpath the shortest
           % path between [S = Starting Node] and [T = Final Node] can be found using
           %[dist, path, pred] = graphshortestpath(G, S, T)
           [dist, path, pred] = graphshortestpath(nNet,1,20,'directed',true);
        end
        
        
        %----------------------------------------------------
        % compute velocity of agents
        % add up forces and compute vx and vy velocity
        %----------------------------------------------------
    end
    
    
    %----------------------------------------------------
    % move agents
    %----------------------------------------------------
    
    
end
%==========================================================================