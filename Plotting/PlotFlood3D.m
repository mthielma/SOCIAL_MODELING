function PlotFlood3D(Parameter,X_Grid,Y_Grid,Z_Grid,time)

ColorFlood_shallow  = [0.0 0.6 1.0];
ColorFlood_deep     = [0.0 0.2 0.8];

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



% if sum(sum(FloodMap))~=0
%     % plot flood
%     hold on
%     scatter3(X_Grid(FloodMap),Y_Grid(FloodMap),FloodHeightMap(FloodMap),Parameter.resolution*10,'filled','MarkerFaceColor',[0.0 0.6 1.0]);
%     hold on
%     scatter3(X_Grid(FloodMap_deep),Y_Grid(FloodMap_deep),FloodHeightMap(FloodMap_deep),Parameter.resolution*20,'filled','MarkerFaceColor',[0.0 0.2 0.8]);
%     hold on
% end


if sum(sum(FloodMap))~=0
    
%     hs=surfl(X_Grid,Y_Grid,FloodHeightMap); set(hs,'EdgeColor','none');
%     colormap('gray')
%     
%     set(hs,'FaceLighting','phong','AmbientStrength',0.3,'DiffuseStrength',0.8,...
%         'SpecularStrength',0.9,'SpecularExponent',25,'BackFaceLighting','lit');


    
%     patch(X_Grid,Y_Grid,FloodHeightMap);
    
    
%     minX = min(min(X_Grid));
%     maxX = max(max(X_Grid));
%     minY = min(min(Y_Grid));
%     maxY = max(max(Y_Grid));
%     minZ = min(min(FloodHeightMap));
%     maxZ = max(max(FloodHeightMap));
% 
%     hp=patch([minX,maxX,minX,maxX], [minY,minY,maxY,maxY], [maxZ,maxZ,maxZ,maxZ], 'b');
%     
%     set(hp,'FaceColor',ColorFlood_shallow,'EdgeColor','none');
%     view(3)
%     hold on

    hold on
    hw=surf(X_Grid,Y_Grid,FloodHeightMap); set(hw,'EdgeColor','none','FaceColor',ColorFlood_shallow,'FaceAlpha',0.5);
%     alpha(.4)
    hold on
    
    hw=surf(X_Grid,Y_Grid,FloodHeightMap_deep); set(hw,'EdgeColor','none','FaceColor',ColorFlood_deep,'FaceAlpha',0.5);
%     alpha(.4)
    hold on

    
end