function PlotAgents3D(XGrid,YGrid,ZGrid,AGENT,BuildingList)

nagents = size(AGENT,2);

radius  = [AGENT.Size];
agent_x = [AGENT.LocX];
agent_y = [AGENT.LocY];
agent_z = interp2(XGrid,YGrid,ZGrid,agent_x,agent_y,'*linear')+radius; % shift z up by radius


figure(99),clf
hold on

% plot surface

surf(XGrid,YGrid,ZGrid),shading interp
meshz(XGrid(1:5:end,1:5:end),YGrid(1:5:end,1:5:end),ZGrid(1:5:end,1:5:end),zeros(size(XGrid(1:5:end,1:5:end))))
colormap('jet')


% plot buildings on surface

for i = 1:size(BuildingList,1)
    % generate patch data for buildings
    x(1) = BuildingList(i,1);
    x(2) = BuildingList(i,1);
    x(3) = BuildingList(i,2);
    x(4) = BuildingList(i,2);
    
    y(1) = BuildingList(i,3);
    y(2) = BuildingList(i,4);
    y(3) = BuildingList(i,4);
    y(4) = BuildingList(i,3);
    
    z    = interp2(XGrid,YGrid,ZGrid,x,y,'*linear');
    patch(x,y,z,'r')
end
freezeColors;

% plot agents
for i = 1:nagents
    [xs,ys,zs] = CreateSphere(radius(i),agent_x(i),agent_y(i),agent_z(i));
    surfl(xs,ys,zs)
    shading flat
end
% plot buildings on surface

for i = 1:size(BuildingList,1)
    % generate patch data for buildings
    x(1) = BuildingList(i,1);
    x(2) = BuildingList(i,1);
    x(3) = BuildingList(i,2);
    x(4) = BuildingList(i,2);
    
    y(1) = BuildingList(i,3);
    y(2) = BuildingList(i,4);
    y(3) = BuildingList(i,4);
    y(4) = BuildingList(i,3);
    
    z    = interp2(XGrid,YGrid,ZGrid,x,y,'*linear');
    patch(x,y,z,'r')
end

axis equal
axis tight
colormap('gray')
az = 88;
el = 15;
view(az, el);
%zoom(3);