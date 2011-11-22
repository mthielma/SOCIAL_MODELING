function PlotAgents3D(XGrid,YGrid,ZGrid,AGENT)

nagents = size(AGENT,2);

radius  = [AGENT.Size];
agent_x = [AGENT.LocX];
agent_y = [AGENT.LocY];
agent_z = interp2(XGrid,YGrid,ZGrid,agent_x,agent_y,'*linear')+radius; % shift z up by radius


figure(1),clf
hold on

% plot surface
meshz(XGrid(1:10:end,1:10:end),YGrid(1:10:end,1:10:end),ZGrid(1:10:end,1:10:end),zeros(size(XGrid(1:10:end,1:10:end)))), shading interp

for i = 1:nagents
    [xs,ys,zs] = CreateSphere(radius(i),agent_x(i),agent_y(i),agent_z(i));
    surfl(xs,ys,zs)
    shading interp
end
axis equal
axis tight
colorbar
az = 49;
el = 16;
view(az, el);
zoom(2);