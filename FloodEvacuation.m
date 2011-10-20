function [STATISTICS, AGENTS] = FloodEvacuation(INPUT,AGENTS,BUILDINGS,STREETS,FLOOD,TOPOGRAPHY)


%%%% FABIO SUPERSTAR !!!!!!!!!!!!!


% initialize buildings
% - location
% - number of people inside
% - number of families inside
% - location of the exit

% initialize graph with nodes, paths between them and cost per path
% - cost due to topography
% - cost due to street width
% - cost due to street type (paved, unpaved)
% - add up all costs

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


%----------------------------------------------------
% compute forces from buildings
%----------------------------------------------------

%----------------------------------------------------
% compute forces from flood
%----------------------------------------------------

%----------------------------------------------------
% compute forces from other agents
%----------------------------------------------------


%----------------------------------------------------
% compute decisions of agents
% - direction
% - help others ?
%----------------------------------------------------


%----------------------------------------------------
% compute velocity of agents
% - generally, each agents wants to go with maximum speed
% - social/wall forces make him slow down
%----------------------------------------------------


%----------------------------------------------------
% move agents
%----------------------------------------------------

%==========================================================================