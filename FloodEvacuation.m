function [STATISTICS, AGENTS] = FloodEvacuation(INPUT,AGENTS,BUILDINGS,STREETS,FLOOD,TOPOGRAPHY)


%%%% FABIO SUPERSTAR !!!!!!!!!!!!!!


%==========================================================================
% initialize buildings
% - location
% - number of people inside
% - number of families inside
% - location of the exit


%----------------------------------------------------
% compute forces from buildings (static)
%----------------------------------------------------


%==========================================================================
% initialize graph with nodes, paths between them and cost per path
% - cost due to topography
% - cost due to street width
% - cost due to street type (paved, unpaved)
% - add up all costs
[X,Y,PathVec,Distance,Gradient] = GenerateRectangularGraph(XGrid,YGrid,ZGrid);
% delete paths at locations where there are buildings


%==========================================================================
% initialize agents in buildings and on streets
% - location
% - mass
% - age -> maximum velocity, max acceleration
% - panic factor (how often are new strategies taken, how much are persons in the way influencing the cost of that path?)
% - horizon distance (how far can the agent look?)
% - group (e.g. family) --> later
% - helping factor --> later


% plot everything



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