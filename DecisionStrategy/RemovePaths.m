function [PathVec,Distance,Gradient,GradientFactor] = RemovePaths(X_Grid,Y_Grid,Map,X,Y,PathVec,Distance,Gradient,GradientFactor)

% remove paths from path vector if paths interfere with buildings or
% something else
%
% Marcel Thielmann, 2011


% get parts of the map where values area actually bigger than 0

[indz,indx] = find(Map>0);

% add 1 index on each side to make sure that a matrix is used to
% interpolate
indz_min = min(indz)-1;
indz_max = max(indz)+1;
indx_min = min(indx)-1;
indx_max = max(indx)+1;

% make sure not to get indices that are too big ot too small
if indz_min < 1, indz_min = 1;end
if indx_min < 1, indx_min = 1;end
if indz_max > size(Map,1), indz_max = size(Map,1);end
if indx_max > size(Map,2), indx_max = size(Map,2);end


MapCut   = Map(indz_min:indz_max,indx_min:indx_max);
XCut     = X_Grid(indz_min:indz_max,indx_min:indx_max);
YCut     = Y_Grid(indz_min:indz_max,indx_min:indx_max);


% interpolate map values from map to X and Y
IsOnSite = interp2(XCut,YCut,MapCut,X,Y,'*linear');
IndOn = find(IsOnSite==1);


% remove all paths that contain a node that is covered by a building
for i = 1:length(IndOn)
    [iz,ix] = find(PathVec == IndOn(i)); % find paths that contain the node number
    PathVec(iz,:)    = []; % remove paths
    Gradient(iz,:)   = [];
    Distance(iz,:)   = [];
    GradientFactor(iz,:)   = [];
end




% remove all paths that go through a building
Remove = zeros(size(PathVec,1),1);
for i = 1:size(PathVec,1)

    nadd  = 10;% add 10 nodes to path for higher resolution
    PathX = zeros(nadd+2,1);
    PathY = zeros(nadd+2,1);
    PathX([1 nadd+2]) = [X(PathVec(i,1)) X(PathVec(i,2))];
    PathY([1 nadd+2]) = [Y(PathVec(i,1)) Y(PathVec(i,2))];
    
    PathX(2:nadd+1) = interp1q([1 nadd+2]',PathX([1 nadd+2]),[2:nadd+1]');
    PathY(2:nadd+1) = interp1q([1 nadd+2]',PathY([1 nadd+2]),[2:nadd+1]');
    
    % just interpolate parts of the map where the path is (add 2m on each side)
    [indz,indx] = find(X_Grid <=max(PathX)+2 & X_Grid >=min(PathX)-2 & Y_Grid <=max(PathY)+2 & Y_Grid >=min(PathY)-2);
    
    indz_min = min(indz);
    indz_max = max(indz);
    indx_min = min(indx);
    indx_max = max(indx);
    
    MapCut   = Map(indz_min:indz_max,indx_min:indx_max);
    XCut     = X_Grid(indz_min:indz_max,indx_min:indx_max);
    YCut     = Y_Grid(indz_min:indz_max,indx_min:indx_max);

    IsSite = interp2(XCut,YCut,MapCut,PathX,PathY,'*linear');
    Remove(i) = max(IsSite);
end

PathVec(Remove>0,:) = [];
Gradient(Remove>0,:) = [];
Distance(Remove>0,:) = [];
GradientFactor(Remove>0,:) = [];