function [Dgradx,Dgrady] = ComputeShortestPathGlobal(BuildingList,Xexit,Yexit,X_Grid,Y_Grid,TopoGradientX,TopoGradientY,v0,resolution)

Debug =false;

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


% this could be done in an iterative manner, but is this necessary?

if sum(TopoGradientX(:))+sum(TopoGradientY(:))>0
    %==============================================================
    % RECOMPUTE WITH TOPOGRAPHY
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
    
    % use fast marching algorithm to compute distance to exit
    [D]=msfm(F, [indx indy]');
    
    % compute gradient in time -> this gives us the direction
    [Dgradx,Dgrady] = gradient(D,resolution,resolution);
    
    % scale direction vectors to normal
    Dgradtot = sqrt(Dgradx.^2+Dgrady.^2);
    Dgradx   = -Dgradx./Dgradtot;
    Dgrady   = -Dgrady./Dgradtot;    
end

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