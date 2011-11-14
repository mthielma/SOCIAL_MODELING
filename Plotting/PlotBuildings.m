function PlotBuildings(BuildingList,Color)

% plot buildings
% BuildingList is a matrix with  [xmin xmax ymin ymax] per row
% Color: FaceColor of the building rectangle
%
% Marcel Thielmann Oct 2011

for i = 1:size(BuildingList,1)
    x = BuildingList(i,1);
    y = BuildingList(i,3);
    w = BuildingList(i,2)-BuildingList(i,1);
    h = BuildingList(i,4)-BuildingList(i,3);
    rectangle('Position',[x y w h],'FaceColor',Color,'EdgeColor',Color);
end