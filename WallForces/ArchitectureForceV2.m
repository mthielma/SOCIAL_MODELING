function [ForceX,ForceY] = ArchitectureForceV2(X_Grid,Y_Grid,List,Parameter,resolution)

% compute architecture forces

% loop through building list and compute the social force from each
% building
% then add up all forces and set forces inside buildings to 0

ForceX = 0*X_Grid;
ForceY = 0*X_Grid;
for i=1:size(List,1)
    MapTemp = zeros(size(X_Grid));
    MapTemp(X_Grid>=List(i,1) & X_Grid<=List(i,2) & Y_Grid>=List(i,3) & Y_Grid<=List(i,4)) = 1;
    % we use bwdist to compute the distance to the first nonzero pixel
    [D,L] = bwdist(MapTemp); 
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
    
    DirX(DirTot==0) = 0;
    DirY(DirTot==0) = 0;
    
    ForceTemp     = Parameter.A.*exp(-D./Parameter.B);
    
    % if Distance bigger than a threshhold, set force 0
    % ForceTemp(D>3) = 0;
    
    ForceTempX    = ForceTemp.*DirX;
    ForceTempY    = ForceTemp.*DirY;
    
    ForceX     = ForceX + ForceTempX;
    ForceY     = ForceY + ForceTempY;
end

% remove forces inside buildings
for i=1:size(List,1)
    ForceX(X_Grid>=List(i,1) & X_Grid<=List(i,2) & Y_Grid>=List(i,3) & Y_Grid<=List(i,4)) = 0;
    ForceY(X_Grid>=List(i,1) & X_Grid<=List(i,2) & Y_Grid>=List(i,3) & Y_Grid<=List(i,4)) = 0;
end

