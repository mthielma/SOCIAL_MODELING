function Agent = DecideOnPathOnCrossing(Agent,PathVec,GradientFactor,Distance,X,Y,Z)

PathWeights = Distance./(Agent.VelMax*GradientFactor);

NewPathVec = PathVec;
NewWeightVec = PathWeights;

% compute the additional cost due to people in the way (here, we have to do
% it for all paths that are adjacent to the node (going away from it)



% set the starting and ending node
StartNode = Agent.DestinationNode;

% generate matrix for shortest path function
PathMat            = sparse(NewPathVec(:,1),NewPathVec(:,2),NewWeightVec(:));

% Find the shortest path via a MATLAB function
% Using the Bioinformatics Toolbox function graphshortestpath the shortest
% path between [S = Starting Node] and [T = Final Node] can be found using
%[dist, path, pred] = graphshortestpath(G, S, T)
time_est = realmax;
for i = 1:length(Agent.ExitNode)
    EndNode   = Agent.ExitNode;
    [time_est2, path, pred] = graphshortestpath(PathMat,StartNode,EndNode,'directed',true);
    if time_est2<time_est
        time_est = time_est2;
        % set the next destination node for the agent
        Agent.DestinationNode = path(2);
        % also switch the path the agent is on
        
    end
end
