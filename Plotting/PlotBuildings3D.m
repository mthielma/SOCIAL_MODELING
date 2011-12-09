function PlotBuildings3D(Parameter,BuildingList,ColorBuildings,Marking)

buildingHeight  = Parameter.AgentSize*1.5;
resolution      = Parameter.resolution;

xmax = Parameter.xmax;
xmin = Parameter.xmin;
ymax = Parameter.ymax;
ymin = Parameter.ymin;

%enlarge field by 1 resolution on each side to completely plot buildings
xvec                = xmin-resolution:resolution:xmax+resolution;
yvec                = ymin-resolution:resolution:ymax+resolution;
[X_Grid,Y_Grid]     = meshgrid(xvec,yvec);

%---------------------------------------
% create building map for later use
%---------------------------------------
BuildingMap = zeros(size(X_Grid));
% add buildings to map
for i=1:size(BuildingList,1)
    BuildingMap(X_Grid>=BuildingList(i,1) & X_Grid<=BuildingList(i,2) & Y_Grid>=BuildingList(i,3) & Y_Grid<=BuildingList(i,4)) = 1;
end
data = BuildingMap;

x2d=zeros(size(data)); y2d=x2d;
for i=1:size(data,1)
    for j=1:size(data,2)
        x2d(i,j) = i;
        y2d(i,j) = j;
    end
end

% figure(2),clf
% surfc(data)
% plot3(x2d,y2d,data,'r')

figure(1)

k=0;
for kk=-1:2  %so building defined from 0 to 1
    k=k+1;
    z1d(k,1) = kk;
end
z1d = z1d*buildingHeight; %enlarge buildings
    
[x3d,y3d,z3d] = meshgrid(xvec,yvec,z1d);

data3d(:,:,2) = data;
data3d(:,:,3) = data;
data3d(:,:,1) = zeros(size(data));
data3d(:,:,4) = zeros(size(data));
% x3d(:,:,1) = x2d;
% x3d(:,:,2) = x2d;
% y3d(:,:,1) = y2d;
% y3d(:,:,2) = y2d;
% z3d(:,:,1) = 0;
% z3d(:,:,2) = 1;

isovalue = 0.9;

[f,v] = isosurface(x3d,y3d,z3d,data3d,isovalue);
p = patch('Faces',f,'Vertices',v);


isonormals(x3d,y3d,z3d,data3d,p)
set(p,'FaceColor',ColorBuildings,'EdgeColor','none');
daspect([1 1 1])
view(3); axis tight
camlight 
lighting gouraud

axis([0 max(xvec) 0 max(yvec) 0 max(z1d)])
hold on