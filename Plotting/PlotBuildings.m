function PlotBuildings(BuildingList,Color,Marking)

% plot buildings
% BuildingList is a matrix with  [xmin xmax ymin ymax] per row
% Color: FaceColor of the building rectangle
%
% Marcel Thielmann Oct 2011

for i = 1:size(BuildingList,1)
    x = BuildingList(i,1);
    x2= BuildingList(i,2);
    y = BuildingList(i,3);
    y2= BuildingList(i,4);
    w = x2-x;
    h = y2-y;
    rectangle('Position',[x y w h],'FaceColor',Color,'EdgeColor',Color);
    if ~strcmp(Marking,'')
       text(x+(x2-x)/2,y2,Marking,'Color','w','FontSize',9,'FontWeight','bold',...
           'VerticalAlignment','top','HorizontalAlignment','center') 
    end
end