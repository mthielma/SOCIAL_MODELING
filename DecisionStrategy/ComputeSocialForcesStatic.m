function [Dgradx,Dgrady] = ComputeShortestPathGlobal(BuildingList,Xexit,Yexit,X_Grid,Y_Grid,Z_Grid)

Debug = false;

% set initial speed map
F = ones(size(X_Grid))*1;
% add buildings to map
for i=1:size(BuildingList,1)
    F(X_Grid>=BuildingList(i,1) & X_Grid<=BuildingList(i,2) & Y_Grid>=BuildingList(i,3) & Y_Grid<=BuildingList(i,4)) = 1e-8;
end

% find indices of exit
[indx,indy] = find(X_Grid==Xexit & Y_Grid==Yexit);

% use fast marching algorithm to compute distance to exit
[D]=msfm(F, [indx indy]');
figure(1),clf

% compute gradient in time -> this gives us the direction
[Dgradx,Dgrady] = gradient(D,resolution,resolution);

% scale direction vectors to normal
Dgradtot = sqrt(Dgradx.^2+Dgrady.^2);
Dgradx   = -Dgradx./Dgradtot;
Dgrady   = -Dgrady./Dgradtot;


% recompute the velocity field based on the slope








if Debug
    
    figure(99),clf
    pcolor(X_Grid,Y_Grid,Dgradx),shading interp
    figure(98),clf
    pcolor(X_Grid,Y_Grid,Dgrady),shading interp
    
    figure(97),clf
    pcolor(X_Grid,Y_Grid,double(D)),shading interp, colorbar
    hold on
    quiver(X_Grid,Y_Grid,Dgradx,Dgrady,'w')
end