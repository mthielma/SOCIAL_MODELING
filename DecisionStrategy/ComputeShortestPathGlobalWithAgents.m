function [Dgradx,Dgrady] = ComputeShortestPathGlobalWithAgents(BuildingList,Xexit,Yexit,X_Grid,Y_Grid,TopoGradientX,TopoGradientY,AGENT,nagent,Parameter)

Debug = false;

% set initial speed map
F = ones(size(X_Grid))*Parameter.v0;
% add buildings to map
for i=1:size(BuildingList,1)
    F(X_Grid>=BuildingList(i,1) & X_Grid<=BuildingList(i,2) & Y_Grid>=BuildingList(i,3) & Y_Grid<=BuildingList(i,4)) = 1e-8;
end

%==============================================================
% FIRST GUESS
%==============================================================

% find indices of exits
ExitPoints = zeros(2,size(Xexit,1));
for i = 1:size(Xexit,1)
    [indx,indy] = find(X_Grid==Xexit(i) & Y_Grid==Yexit(i));
    ExitPoints(1,i) = indx;
    ExitPoints(2,i) = indy;
end
% use fast marching algorithm to compute distance to exit
[D_orig]=msfm(F, ExitPoints);

% compute gradient in time -> this gives us the direction
[Dgradx_orig,Dgrady_orig] = gradient(D_orig,Parameter.resolution,Parameter.resolution);

% scale direction vectors to normal
Dgradtot = sqrt(Dgradx_orig.^2+Dgrady_orig.^2);
Dgradx_orig   = -Dgradx_orig./Dgradtot;
Dgrady_orig   = -Dgrady_orig./Dgradtot;

%==============================================================
% RECOMPUTE WITH TOPOGRAPHY AND AGENTS
%==============================================================
% recompute the velocity field based on the slope
slope = TopoGradientX.*Dgradx_orig+TopoGradientY.*Dgrady_orig;

% compute maximum velocity due to slope
PreFac = (Parameter.v0./exp(-3.5*0.05));
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
   F(Inside<AGENT(iagent).Size) = F0(Inside<AGENT(iagent).Size)*0.25;
end


% use fast marching algorithm to compute distance to exit
[D_agent]=msfm(F, ExitPoints);

% compute mixture of both fields taking into account sensitivity to agents
% in the way
Dmix = (D_orig + Parameter.agent_sensitivity.*D_agent)./(Parameter.agent_sensitivity+1);
[Dgradx,Dgrady] = gradient(Dmix,Parameter.resolution,Parameter.resolution);

Dgradtot = sqrt(Dgradx.^2+Dgrady.^2);
Dgradx   = -Dgradx./Dgradtot;
Dgrady   = -Dgrady./Dgradtot;

if Debug

    figure(99),clf
    pcolor(X_Grid,Y_Grid,double(D)),shading interp, colorbar
    hold on
    quiver(X_Grid,Y_Grid,Dgradx,Dgrady,'w')
    
    figure(98),clf
    pcolor(X_Grid,Y_Grid,double(D)),shading interp, colorbar
    hold on
    quiver(X_Grid,Y_Grid,Dgradx_agent,Dgrady_agent,'w')
    
    figure(97),clf
    pcolor(X_Grid,Y_Grid,double(D)),shading interp, colorbar
    hold on
    quiver(X_Grid,Y_Grid,Dgradx_orig,Dgrady_orig,'w')
    
    
    
end