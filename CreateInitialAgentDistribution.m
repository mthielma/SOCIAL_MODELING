function AGENT = CreateInitialAgentDistribution(nagent,AGENT,X_Grid,Y_Grid,BuildingMap,BoundaryMap,StartArea,ExitMap)

% get the indices in BuildingMap that correspond to a road
[iy,ix] = find(StartArea==1 & BoundaryMap==0 & BuildingMap==0 & ExitMap==0);
NoRoad  = length(iy);
% create random indices (make sure that there are no duplicates)
indices = randperm(NoRoad);

iyStart = iy(indices(1:nagent));
ixStart = ix(indices(1:nagent));

indices = sub2ind(size(X_Grid),iyStart,ixStart);

StartLocX = X_Grid(indices);
StartLocY = Y_Grid(indices);

cellX = num2cell(StartLocX);
cellY = num2cell(StartLocY);
[AGENT(1:nagent).LocX]       = cellX{:};
[AGENT(1:nagent).LocY]       = cellY{:};