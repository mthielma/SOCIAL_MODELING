function PlotAgents(AGENT,Plotting)
% plot agents as circles with the size of the circle being their radius
% 
% 
% AGENT is a structure that needs to have the following fields:
% .Size: radius of the agent
% .LocX: x-location
% .LocY: y-location
%
% Plotting is a structure that needs to have the following fields:
% .Marking: defines agents description
% .Color: face color of the marker
%
% Marcel Thielmann Oct 2011

nagent = size(AGENT,2);
for i = 1:nagent
    radius = AGENT(i).Size;
    x      = AGENT(i).LocX;
    y      = AGENT(i).LocY;
    name   = AGENT(i).name;
    try
    if strcmp(Plotting.Color,'rand') ||  strcmp(Plotting.Color,'one'); %random color   or   one coloured agent
        rectangle('position',[x-radius, y-radius, 2*radius, 2*radius],'curvature',[1 1],'FaceColor',Plotting.cmap(name,:));
    else %all the same color
        rectangle('position',[x-radius, y-radius, 2*radius, 2*radius],'curvature',[1 1],'FaceColor',Plotting.Color);
    end
    catch
        bla=1;
    end
    
    if strcmp(Plotting.Marking,'none');
    elseif strcmp(Plotting.Marking,'number'); 
        agentText = name;
        text(x,y,agentText,'HorizontalAlignment','center','VerticalAlignment','middle','FontSize',10)
    elseif strcmp(Plotting.Marking,'smiley'); 
        agentText = ':-)';
        text(x,y,agentText,'HorizontalAlignment','center','VerticalAlignment','middle','FontSize',12)
    end
    
end