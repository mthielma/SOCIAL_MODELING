function AGENT = CheckAgentsInBuildings(AGENT,BuildingList,X_Grid,Y_Grid,ArchDirX,ArchDirY,ArchD)



% loop through building list and find out if agent is inside building
in_building = zeros(size(AGENT));
for i=1:size(BuildingList,1)
    
    % find agents in polygon
    x1 = BuildingList(i,1);
    x2 = BuildingList(i,1);
    x3 = BuildingList(i,2);
    x4 = BuildingList(i,2);
    
    y1 = BuildingList(i,3);
    y2 = BuildingList(i,4);
    y3 = BuildingList(i,4);
    y4 = BuildingList(i,3);
    
    Polyx = [x1 x2 x3 x4];
    Polyy = [y1 y2 y3 y4];
    
    [IN,ON] = inpolygon([AGENT.LocX],[AGENT.LocY],Polyx,Polyy);
    
    
    in_building(IN) = 1;
    in_building(ON) = 1;
end

% move agent out from building

% I. interpolate distance to walls from inside to agents
DistToWall = interp2(X_Grid,Y_Grid,ArchD,[AGENT(in_building==1).LocX],[AGENT(in_building==1).LocY],'*linear');
XDir       = interp2(X_Grid,Y_Grid,ArchDirX,[AGENT(in_building==1).LocX],[AGENT(in_building==1).LocY],'*linear');
YDir       = interp2(X_Grid,Y_Grid,ArchDirY,[AGENT(in_building==1).LocX],[AGENT(in_building==1).LocY],'*linear');



% II. add distance to wall to location of agent
AgentShiftX = DistToWall.*XDir + [AGENT(in_building==1).Size]./2;
AgentShiftY = DistToWall.*YDir + [AGENT(in_building==1).Size]./2;

NewLocX = num2cell([AGENT(in_building==1).LocX] +AgentShiftX);
NewLocY = num2cell([AGENT(in_building==1).LocY] +AgentShiftY);

[AGENT((in_building==1)).LocX] = NewLocX{:};
[AGENT((in_building==1)).LocY] = NewLocY{:};
