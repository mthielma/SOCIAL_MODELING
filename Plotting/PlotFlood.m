function PlotFlood(Parameter,X_Grid,Y_Grid,Z_Grid,time)



dzdt_flood = Parameter.dzdt_flood;
dangerousDepth  = Parameter.dangerousDepth;


%rise flood
Z_flood = Parameter.z0_flood + dzdt_flood*time;


%----------------------------------------------------
% create flood map
%----------------------------------------------------
%compute height
Z_flood_deep = Z_flood - dangerousDepth;

%create floodmap
FloodMap = logical(X_Grid*0); FloodMap_deep = FloodMap;
FloodHeightMap = ones(size(Z_Grid))*Z_flood;

FloodHeightMap_deep = ones(size(Z_Grid))*Z_flood_deep;
FloodMap_deep(FloodHeightMap_deep>Z_Grid) = 1;

FloodMap( FloodHeightMap>Z_Grid & FloodHeightMap_deep<Z_Grid ) = 1;



if sum(sum(FloodMap))~=0
    % plot flood
    hold on
    scatter(X_Grid(FloodMap),Y_Grid(FloodMap),Parameter.resolution*10,'filled','MarkerFaceColor',[0.0 0.6 1.0]);
    hold on
    scatter(X_Grid(FloodMap_deep),Y_Grid(FloodMap_deep),Parameter.resolution*20,'filled','MarkerFaceColor',[0.0 0.2 0.8]);
    hold on
end