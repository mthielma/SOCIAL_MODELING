function [FxPhysAgents,FyPhysAgents] = ComputePhysicalForceAgents(velx_agent,vely_agent,velx_others,vely_others,Parameter,DistanceToAgents,Normal,Tangent)

% normal force
F_physAgents_normalX = Parameter.k.*DistanceToAgents.*Normal(:,1);
F_physAgents_normalY = Parameter.k.*DistanceToAgents.*Normal(:,2);
% tangential force
DeltaV      = (velx_others-velx_agent).*Tangent(:,1)+(vely_others-vely_agent).*Tangent(:,2);
F_physAgents_tangentX = Parameter.kappa.*DistanceToAgents.*DeltaV.*Tangent(:,1);
F_physAgents_tangentY = Parameter.kappa.*DistanceToAgents.*DeltaV.*Tangent(:,2);
% add physical forces
FxPhysAgents = F_physAgents_normalX + F_physAgents_tangentX;
FyPhysAgents = F_physAgents_normalY + F_physAgents_tangentY; 