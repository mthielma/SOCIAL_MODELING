function [PathVec,Distance,Gradient] = RemovePaths(X_Grid,Y_Grid,Map,X,Y,PathVec,Distance,Gradient)

% remove paths from path vector if paths interfere with buildings or
% something else
%
% Marcel Thielmann, 2011

% interpolate map values from building map to X and Y
IsOnSite = interp2(X_Grid,Y_Grid,Map,X,Y,'nearest'); % NN-interpolation
% remove all paths that contain a node that is covered by a building
IndOn = find(IsOnSite==1);

for i = 1:length(IndOn)
   [iz,ix] = find(PathVec == IndOn(i)); % find paths that contain the node number
   PathVec(iz,:)    = []; % remove paths
   Gradient(iz,:)   = [];
   Distance(iz,:)   = [];
end

axis equal, axis tight


% remove all paths that go through a building
Remove = zeros(size(PathVec,1),1);
for i = 1:size(PathVec,1)

    nadd  = 10;% add 10 nodes to path for higher resolution
    PathX = zeros(nadd+2,1);
    PathY = zeros(nadd+2,1);
    PathX([1 nadd+2]) = [X(PathVec(i,1)) X(PathVec(i,2))];
    PathY([1 nadd+2]) = [Y(PathVec(i,1)) Y(PathVec(i,2))];
    
    PathX = interp1([1 nadd+2],PathX([1 nadd+2]),1:nadd+2);
    PathY = interp1([1 nadd+2],PathY([1 nadd+2]),1:nadd+2);
    
    IsSite = interp2(X_Grid,Y_Grid,Map,PathX,PathY,'nearest');
    Remove(i) = max(IsSite);
end

PathVec(Remove>0,:) = [];
Gradient(Remove>0,:) = [];
Distance(Remove>0,:) = [];