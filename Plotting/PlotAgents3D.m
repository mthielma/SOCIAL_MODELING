function PlotAgents3D_2(Parameter,Plotting,AGENT)

xmax = Parameter.xmax;
xmin = Parameter.xmin;
ymax = Parameter.ymax;
ymin = Parameter.ymin;

xvec                = xmin:Parameter.resolution:xmax;
yvec                = ymin:Parameter.resolution:ymax;
[X_Grid,Y_Grid]     = meshgrid(xvec,yvec);

Z_Grid = zeros(size(X_Grid));

nagents = size(AGENT,2);

% plot agents
for i = 1:nagents
    radius  = [AGENT(i).Size];
    agent_x	= [AGENT(i).LocX];
    agent_y	= [AGENT(i).LocY];
    name    = [AGENT(i).name];
    agent_z = interp2(X_Grid,Y_Grid,Z_Grid,agent_x,agent_y,'*linear')+radius; % shift z up by radius
    
    figure(1); hold on
    [xs,ys,zs] = CreateSphere(radius,agent_x,agent_y,agent_z);
    
    if strcmp(Plotting.Color,'rand') ||  strcmp(Plotting.Color,'one'); %random color   or   one coloured agent
        hs=surfl(xs,ys,zs); set(hs,'EdgeColor','none','FaceColor',Plotting.cmap(name,:));
    else %all the same color
        hs=surfl(xs,ys,zs); set(hs,'EdgeColor','none','FaceColor',Plotting.Color);
    end
    set(hs,'FaceLighting','phong','AmbientStrength',0.3,'DiffuseStrength',0.8,...
        'SpecularStrength',0.9,'SpecularExponent',25,'BackFaceLighting','lit');
%     camlight left;
%     shading flat
end

% axis equal
% axis tight
% colormap('gray')
% az = 88;
% el = 15;
% view(az, el);
%zoom(3);