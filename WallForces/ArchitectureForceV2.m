function [ArchForce,DirX,DirY] = ArchitectureForceV2(X_Grid,Y_Grid,Map,Parameter,resolution)

% compute architecture forces

% loop through building list and compute the social force from each
% building
% then add up all forces and set forces inside buildings to 0

[D,L] = bwdist(Map);

D = double(D).*resolution; % distance is given in pixels, which is why we have to transform it to real distances
% compute the direction vector from each pixel to its nearest nonzero
% pixel
DirX = 0*X_Grid;
DirY = 0*X_Grid;
DirX(:) = X_Grid(:) - X_Grid(L(:)); % actual distance between actual point and nearest building point
DirY(:) = Y_Grid(:) - Y_Grid(L(:)); % actual distance between actual point and nearest building point

DirTot = sqrt(DirX.*DirX+DirY.*DirY);
DirX = DirX./DirTot;
DirY = DirY./DirTot;

ArchForce   = Parameter.A.*exp(-D./Parameter.B);
% buildings
ArchForce(logical(Map)) = 0;


% invert map
MapInv = ~(logical(Map));
[D,L] = bwdist(MapInv);
D = double(D).*resolution;

DirX_B = 0*X_Grid;
DirY_B = 0*X_Grid;
DirX_B(:) = -(X_Grid(:) - X_Grid(L(:))); % actual distance between actual point and nearest building point
DirY_B(:) = -(Y_Grid(:) - Y_Grid(L(:))); % actual distance between actual point and nearest building point

DirTot_B = sqrt(DirX_B.*DirX_B+DirY_B.*DirY_B);
DirX_B = DirX_B./DirTot_B;
DirY_B = DirY_B./DirTot_B;


% treat buildings
ArchForce(logical(Map)) = max(ArchForce(:));

DirX(logical(Map)) = DirX_B(logical(Map));
DirY(logical(Map)) = DirY_B(logical(Map));



