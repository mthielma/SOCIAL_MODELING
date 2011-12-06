function [FxPhysWall,FyPhysWall] = ComputePhysicalForceWalls(x_agent,y_agent,agent_size,velx_agent,vely_agent,x_building,y_building,Parameter)

% get minimum distance to another wall
xdist = x_building-x_agent;
ydist = y_building-y_agent;

WallDist2             =    (xdist).*(xdist)+(ydist).*(ydist); 	% squared distance between agent's center of mass and wall boundary
[minWallDist2,ind]     = min(WallDist2);
WallDist              = -sqrt(minWallDist2)+agent_size;                            %between agent's boundary and wall boundary

x_building = x_building(ind);
y_building = y_building(ind);

% compute normal and tangential vector
Normal(:,1)         = (x_agent - x_building)./minWallDist2;  %DistanceToAgents should not be zero!
Normal(:,2)         = (y_agent - y_building)./minWallDist2;

Tangent(:,1)         = -Normal(:,2);
Tangent(:,2)         = Normal(:,1);

if WallDist>=0
    % normal force
    F_physWall_normalX = 3*Parameter.k.*WallDist.*Normal(:,1);
    F_physWall_normalY = 3*Parameter.k.*WallDist.*Normal(:,2);
    
    if Parameter.Tangential
        % tangential force
        DeltaV      = (-velx_agent).*Tangent(:,1)+(-vely_agent).*Tangent(:,2);
        F_physWall_tangentX = Parameter.kappa.*WallDist.*DeltaV.*Tangent(:,1);
        F_physWall_tangentY = Parameter.kappa.*WallDist.*DeltaV.*Tangent(:,2);
    else
        F_physWall_tangentX = 0;
        F_physWall_tangentY = 0;
    end
    % add physical forces
    FxPhysWall = F_physWall_normalX + F_physWall_tangentX;
    FyPhysWall = F_physWall_normalY + F_physWall_tangentY;
else
    FxPhysWall = 0;
    FyPhysWall = 0;
end


