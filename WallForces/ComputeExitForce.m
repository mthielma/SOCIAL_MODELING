function [AGENT] = ComputeExitForce(AGENT,Parameter,nagent)
% idea: each individual wants to get to the exit more than it is held up by
% the social forces from walls and other agents


% add up social forces from walls and other agents
social_force = sqrt(([AGENT.FxSocialWalls]+[AGENT.FxSocialAgents]).^2+([AGENT.FySocialWalls]+[AGENT.FySocialAgents]).^2);

% compute the force from the exit
xForceExit = [AGENT(1:nagent).xExitDir].*social_force.*Parameter.ExitFactor;
yForceExit = [AGENT(1:nagent).yExitDir].*social_force.*Parameter.ExitFactor;

% set forces to a constant value inside walls to get agents out of the wall
xForceExit(xForceExit==0) = [AGENT((xForceExit==0)).xExitDir].*1e5;
yForceExit(yForceExit==0) = [AGENT((yForceExit==0)).yExitDir].*1e5;

dummy = num2cell(xForceExit);
[AGENT.xForceExit] = dummy{:};
dummy = num2cell(yForceExit);
[AGENT.yForceExit] = dummy{:};