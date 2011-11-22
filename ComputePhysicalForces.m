function [FxDynamicSocial,FyDynamicSocial,DistanceToAgents] = ComputeSocialForcesDynamic(x_agent,y_agent,agent_size,x_others,y_others,others_size,Parameter,NormalVector)

% compute distance between agents
DistanceToAgents    = sqrt((x_agent-x_others).^2 + (y_agent-y_others).^2) - agent_size - others_size;

% cmpute social force
F_socAgents     = Parameter.A.*exp(DistanceToAgents)./Parameter.B;

FxDynamicSocial    = F_socAgents.*NormalVector(1);
FyDynamicSocial    = F_socAgents.*NormalVector(2);

