function Agent = DecideOnPath(Agent,PathVec,GradientFactor,Distance,X,Y,Z)

PathWeights = Distance./(Agent.VelMax*GradientFactor);

% add the paths between the agent and the next crossings (nodes)
% to the path vector
maxNode   = max(PathVec(:));
NodeAgent = maxNode+1;
NodeEnd   = PathVec(Agent.PresentPath,2);
NodeStart = PathVec(Agent.PresentPath,1);

DistanceEnd      = sqrt((Agent.LocX-X(NodeEnd)).^2+(Agent.LocY-Y(NodeEnd)).^2);
DistanceStart    = sqrt((Agent.LocX-X(NodeStart)).^2+(Agent.LocY-Y(NodeStart)).^2);
GradientEnd      = Agent.LocZ-Z(NodeEnd);
GradientStart    = Agent.LocZ-Z(NodeStart);

GradientFactorEnd   = exp(-3.5*abs(GradientEnd+0.05));
GradientFactorStart = exp(-3.5*abs(GradientStart+0.05));

WeightEnd        = DistanceEnd./(Agent.VelMax*GradientFactorEnd);
WeightStart      = DistanceStart./(Agent.VelMax*GradientFactorStart);

% compute the additional cost due to people in the way


% add the two paths and weights to the path vector
NewPathVec       = [PathVec;[NodeAgent NodeEnd];[NodeAgent NodeStart]];
NewWeightVec     = [PathWeights;WeightEnd;WeightStart];
% set the starting and ending node
StartNode = NodeAgent;

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
    end
end
