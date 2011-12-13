function [Dgradx,Dgrady,D] = ComputeShortestPathGlobal(FloodMap,FloodMap_deep,BuildingMap,BuildingMap_boundary,ExitMap,X_Grid,Y_Grid,Parameter)

Debug =false;
resolution  = Parameter.resolution;
v0          = Parameter.v0;

% set initial speed map
F = ones(size(X_Grid))*v0;

%add building boundaries to map
F(BuildingMap_boundary==1) = v0/3;
% add flood to map
F(FloodMap) = Parameter.FloodSpeed;
F(FloodMap_deep) = 1e-8;
% add buildings to map
F(BuildingMap) = 1e-8;


%==============================================================
% Shortest path w/o topo
%==============================================================

% find indices of exits
[indx,indy] = find(ExitMap == 1);
ExitPoints = [indx';indy'];

% use fast marching algorithm to compute distance to exit
[D]=msfm(F, ExitPoints);

% compute gradient in time -> this gives us the direction
[Dgradx,Dgrady] = gradient(D,resolution,resolution);

% scale direction vectors to normal
Dgradtot = sqrt(Dgradx.^2+Dgrady.^2);
Dgradx   = -Dgradx./Dgradtot;
Dgrady   = -Dgrady./Dgradtot;

% this could be done in an iterative manner, but is this necessary?

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