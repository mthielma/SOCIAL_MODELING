function [FxDynamicSocial,FyDynamicSocial] = ComputeSocialForcesDynamic(Parameter,DistanceToAgents,NormalVector)

% cmpute social force
F_socAgents     = Parameter.A.*exp(DistanceToAgents./Parameter.B);

FxDynamicSocial    = F_socAgents.*NormalVector(:,1);
FyDynamicSocial    = F_socAgents.*NormalVector(:,2);

