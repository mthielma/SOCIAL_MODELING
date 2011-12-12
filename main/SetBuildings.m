% function to generate the building list for the beach setup
function BuildingList = SetBuildings(DomainWidth,DomainHeight,StreetWidth,BuildingWidthX,BuildingWidthY)



% x-direction
i = 1;
x_min_vec(i) = 0;
x_max_vec(i) = x_min_vec(i)+BuildingWidthX;

while x_min_vec(i)<DomainWidth
    i=i+1;
    x_min_vec(i) = x_min_vec(i-1)+BuildingWidthX+StreetWidth;
    x_max_vec(i) = x_max_vec(i-1)+BuildingWidthX+StreetWidth;
    
    
    
    if x_max_vec(i) > DomainWidth
        x_max_vec(i) = DomainWidth;
    end
end

if x_min_vec(end) >= x_max_vec(end)
    x_min_vec(end) = [];
    x_max_vec(end) = [];
end
% y-direction

i = 1;
y_min_vec(i) = 0;
y_max_vec(i) = y_min_vec(i)+BuildingWidthY;

while y_min_vec(i)<DomainHeight
    i=i+1;
    y_min_vec(i) = y_min_vec(i-1)+BuildingWidthY+StreetWidth;
    y_max_vec(i) = y_max_vec(i-1)+BuildingWidthY+StreetWidth;
    if y_max_vec(i) > DomainHeight
        y_max_vec(i) = DomainHeight;
    end
end
if y_min_vec(end) >= y_max_vec(end) 
    y_min_vec(end) = [];
    y_max_vec(end) = [];
end


[XMIN,YMIN] = meshgrid(x_min_vec,y_min_vec);
[XMAX,YMAX] = meshgrid(x_max_vec,y_max_vec);


BuildingList = [XMIN(:) XMAX(:) YMIN(:) YMAX(:)];
