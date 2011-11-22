function [Normal,Tangent,DistanceToAgents,num_others] = ComputeDistanceToAgents(x_agent,y_agent,agent_size,x_others,y_others,others_size)

num_others = length(x_others);
if num_others == 0
    Normal = [];
    Tangent = [];
    DistanceToAgents = [];
else
    Normal      = zeros(num_others,2);
    Tangent     = zeros(num_others,2);
    
    % compute distance between agents
    MassCenterDistance  = sqrt((x_agent-x_others).^2 + (y_agent-y_others).^2);
    % DistanceToAgents    = MassCenterDistance - agent_size - others_size;
    DistanceToAgents    = (agent_size+others_size) - MassCenterDistance;
    % compute normal and tangential vector between agents
    % compute normal vector
    Normal(:,1)         = (x_agent - x_others)./DistanceToAgents;  %DistanceToAgents should not be zero!
    Normal(:,2)         = (y_agent - y_others)./DistanceToAgents;
    
    Tangent(:,1)         = -Normal(:,2);
    Tangent(:,2)         = Normal(:,1);
end