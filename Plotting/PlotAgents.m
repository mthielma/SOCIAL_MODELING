function PlotAgents(nagent,AGENT,Color)
% plot agents as circles with the size of the circle being their radius
% AGENT is a structure that needs to have the following fields:
% .Size: radius of the agent
% .LocX: x-location
% .LocY: y-location
%
% nagent: number of agents
% Color: face color of the marker
%
% Marcel Thielmann Oct 2011

for i = 1:nagent
    radius = AGENT(i).Size/2;
    x      = AGENT(i).LocX;
    y      = AGENT(i).LocY;
    try
    rectangle('position',[x-radius, y-radius, 2*radius, 2*radius],'curvature',[1 1],'FaceColor',Color);
    catch
        bla =1;
    end
end