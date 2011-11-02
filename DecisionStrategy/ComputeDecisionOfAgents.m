function AGENT = ComputeDecisionOfAgents()


%----------------------------------------------------
%compute decisions of agents
% - in case of extreme slowdown
% - the agent has reached the destination point
% - the destination point or a part of the path are flooded
%----------------------------------------------------

DistanceDestinationPoint
% has the agent reached the destination point?


% is the destination point flooded?


% see if slowdown causes agent to rethink his exit strategy
if AGENT(iagent).Velocity/AGENT(iagent).VMax < AGENT(iagent).MaxSlowdown
    AGENT(iagent).SlowdownSteps = AGENT(iagent).SlowdownSteps+1;
    if AGENT(iagent).SlowdownSteps > AGENT(iagent).MaxSlowdownSteps % agent is fed up because of slowdown
        Decision        = true;
        DecisionCause   = 1;
    else
        Decision = false;
    end
else
    AGENT(iagent).SlowdownSteps = 0; % reset if the agent can move with a satifying velocity
    Decision = false;
end




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