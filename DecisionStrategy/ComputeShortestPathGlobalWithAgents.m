function [Dgradx,Dgrady] = ComputeShortestPathGlobalWithAgents(BuildingList,Xexit,Yexit,X_Grid,Y_Grid,TopoGradientX,TopoGradientY,v0,resolution,AGENT,nagent,agent_sensitivity)

Debug = false;

% set initial speed map
F = ones(size(X_Grid))*v0;
% add buildings to map
for i=1:size(BuildingList,1)
    F(X_Grid>=BuildingList(i,1) & X_Grid<=BuildingList(i,2) & Y_Grid>=BuildingList(i,3) & Y_Grid<=BuildingList(i,4)) = 1e-8;
end

%==============================================================
% FIRST GUESS
%==============================================================

% find indices of exit
[indx,indy] = find(X_Grid==Xexit & Y_Grid==Yexit);

% use fast marching algorithm to compute distance to exit
[D]=msfm(F, [indx indy]');

% compute gradient in time -> this gives us the direction
[Dgradx,Dgrady] = gradient(D,resolution,resolution);

% scale direction vectors to normal
Dgradtot = sqrt(Dgradx.^2+Dgrady.^2);
Dgradx   = -Dgradx./Dgradtot;
Dgrady   = -Dgrady./Dgradtot;

%==============================================================
% RECOMPUTE WITH TOPOGRAPHY AND AGENTS
%==============================================================
% recompute the velocity field based on the slope
slope = TopoGradientX.*Dgradx+TopoGradientY.*Dgrady;

% compute maximum velocity due to slope
PreFac = (v0./exp(-3.5*0.05));
F = PreFac.*exp(-3.5.*abs(slope+0.05));
% add buildings to map
for i=1:size(BuildingList,1)
    F(X_Grid>=BuildingList(i,1) & X_Grid<=BuildingList(i,2) & Y_Grid>=BuildingList(i,3) & Y_Grid<=BuildingList(i,4)) = 1e-8;
end

F0 = F;
% find out at which node agents are located and reduce velocity at this
% node by a constant factor (for now reduce it to 10 % of the actual velocity)
% unfortunately, we have to loop over the agents so far...
for iagent = 1:nagent
   Inside = (X_Grid-AGENT(iagent).LocX).*(X_Grid-AGENT(iagent).LocX)+(Y_Grid-AGENT(iagent).LocY).*(Y_Grid-AGENT(iagent).LocY);
   F(Inside<AGENT(iagent).Size) = F0(Inside<AGENT(iagent).Size)*agent_sensitivity;
end


% use fast marching algorithm to compute distance to exit
[D]=msfm(F, [indx indy]');

% compute gradient in time -> this gives us the direction
[Dgradx,Dgrady] = gradient(D,resolution,resolution);

% scale direction vectors to normal
Dgradtot = sqrt(Dgradx.^2+Dgrady.^2);
Dgradx   = -Dgradx./Dgradtot;
Dgrady   = -Dgrady./Dgradtot;








if Debug
    
    figure(99),clf
    pcolor(X_Grid,Y_Grid,Dgradx),shading interp
    figure(98),clf
    pcolor(X_Grid,Y_Grid,Dgrady),shading interp
    
    figure(97),clf
    pcolor(X_Grid,Y_Grid,double(D)),shading interp, colorbar
    hold on
    quiver(X_Grid,Y_Grid,Dgradx,Dgrady,'w')
    pause
end