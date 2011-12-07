function [Dgradx,Dgrady] = ComputeShortestPathGlobalDirect(BuildingMap,ExitMap,X_Grid,Y_Grid,v0,resolution)

Debug =false;

% set initial speed map
F = ones(size(X_Grid))*v0;

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


% invert map
MapInv = ~(logical(BuildingMap));
[D,L] = bwdist(MapInv);

DirX_B = 0*X_Grid;
DirY_B = 0*X_Grid;
DirX_B(:) = -(X_Grid(:) - X_Grid(L(:))); % actual distance between actual point and nearest building point
DirY_B(:) = -(Y_Grid(:) - Y_Grid(L(:))); % actual distance between actual point and nearest building point

DirTot_B = sqrt(DirX_B.*DirX_B+DirY_B.*DirY_B);
DirX_B = DirX_B./DirTot_B;
DirY_B = DirY_B./DirTot_B;

Dgradx(logical(BuildingMap)) = DirX_B(logical(BuildingMap));
Dgrady(logical(BuildingMap)) = DirY_B(logical(BuildingMap));







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