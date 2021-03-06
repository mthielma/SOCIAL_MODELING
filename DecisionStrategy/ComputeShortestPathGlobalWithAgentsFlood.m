
function [Dgradx,Dgrady] = ComputeShortestPathGlobalWithAgentsFlood(FloodMap,FloodMap_deep,BuildingMap,BuildingMap_boundary,ExitMap,X_Grid,Y_Grid,D_orig,Dgradx,Dgrady,TopoGradientX,TopoGradientY,AGENT,nagent,Parameter)
Debug = false;

FloodMap    = logical(FloodMap);
BuildingMap = logical(BuildingMap);
% set initial speed map
F_agent = ones(size(X_Grid))*Parameter.v0;

% add building boundaries to map
F_agent(BuildingMap_boundary==1) = Parameter.v0/3;
% consider topo
% recompute the velocity field based on the slope
slope = TopoGradientX.*Dgradx+TopoGradientY.*Dgrady;

% compute maximum velocity due to slope
PreFac = (Parameter.v0./exp(-Parameter.slope_f*Parameter.slope_crit));
F_agent = PreFac.*exp(-Parameter.slope_f.*abs(slope+Parameter.slope_crit));

% add flood to map
F_agent(FloodMap) = Parameter.FloodSpeed;
F_agent(FloodMap_deep) = 1e-8;

% add buildings to map
F_agent(BuildingMap) = 1e-8;


%==============================================================
% Exits
%==============================================================

% find indices of exits
[indx,indy] = find(ExitMap == 1);
ExitPoints = [indx';indy'];

%==============================================================
% COMPUTE WITH AGENTS
%==============================================================
% find out at which node agents are located and reduce velocity at this
% node by a constant factor (for now reduce it to 25 % of the actual velocity)
% unfortunately, we have to loop over the agents so far...
for iagent = 1:nagent
   Inside = (X_Grid-AGENT(iagent).LocX).*(X_Grid-AGENT(iagent).LocX)+(Y_Grid-AGENT(iagent).LocY).*(Y_Grid-AGENT(iagent).LocY);
   F_agent(Inside<AGENT(iagent).Size) = F_agent(Inside<AGENT(iagent).Size)*1/Parameter.agent_sensitivity;
end

% use fast marching algorithm to compute distance to exit
[D_agent]=msfm(F_agent, ExitPoints);
%[D_test] = msfm2d_gradient(Z_Grid,Parameter.resolution,Parameter, ExitPoints,false,false);



% compute gradients of both fields and add the directions
[Dgradx_bg,Dgrady_bg] = gradient(D_orig,Parameter.resolution,Parameter.resolution);
[Dgradx,Dgrady] = gradient(D_agent,Parameter.resolution,Parameter.resolution);


Dgradx = Dgradx_bg+Parameter.orig_sensitivity.*Dgradx;
Dgrady = Dgrady_bg+Parameter.orig_sensitivity.*Dgrady;

Dgradtot = sqrt(Dgradx.^2+Dgrady.^2);
Dgradx   = -Dgradx./Dgradtot;
Dgrady   = -Dgrady./Dgradtot;

% scale d_orig and D_agent to their respective max
% D_orig(~BuildingMap) = D_orig(~BuildingMap)./max(D_orig(~BuildingMap(:)));
% D_agent(~BuildingMap) = D_agent(~BuildingMap)./max(D_agent(~BuildingMap(:)));

% compute mixture of both fields taking into account sensitivity to agents
% in the way
Dmix = (D_agent+ Parameter.orig_sensitivity.*D_orig);

[Dgradx,Dgrady] = gradient(Dmix,Parameter.resolution,Parameter.resolution);

Dgradtot = sqrt(Dgradx.^2+Dgrady.^2);
Dgradx   = -Dgradx./Dgradtot;
Dgrady   = -Dgrady./Dgradtot;


if Debug

    figure(99),clf
    subplot(131)
    hold on
    pcolor(X_Grid,Y_Grid,D_orig),shading flat, colorbar
    %quiver(X_Grid,Y_Grid,Dgradx_orig,Dgrady_orig)
    caxis([0 1])
    axis equal,axis tight
    subplot(132)
    hold on
    pcolor(X_Grid,Y_Grid,D_topo),shading flat, colorbar
    %quiver(X_Grid,Y_Grid,Dgradx_topo,Dgrady_topo)
     axis equal,axis tight
    caxis([0 1])
    subplot(133)
    hold on
    pcolor(X_Grid,Y_Grid,D_agent),shading flat, colorbar
    %quiver(X_Grid,Y_Grid,Dgradx_agent,Dgrady_agent)
     axis equal,axis tight
    caxis([0 1])
     
    figure(98),clf
    pcolor(X_Grid,Y_Grid,Dmix-D_orig),shading flat, colorbar
    caxis([0 5])
    hold on
    quiver(X_Grid,Y_Grid,Dgradx,Dgrady,'w')
    
     axis equal,axis tight
end