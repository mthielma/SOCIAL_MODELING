function [X,Y,PathVec,Distance,Gradient] = GenerateRectangularGraph(XGrid,YGrid,ZGrid)
% generate graph for a rectangular net of streets.
% additionally compute topography differences
%
% Marcel Thielmann, Oct 2011


Debug = 0; % default is no debugging
% INPUT: rectangular grids for x,y,z
if nargin == 0
   Debug = 1;
   xvec = 0:5;
   yvec = 0:5;
   [XGrid,YGrid] = meshgrid(xvec,yvec);
   ZGrid         = XGrid*0;
end

[Ny,Nx] = size(XGrid);


%------------------------------
% create coordinate vectors
% use matlabs linear indexing for that
%------------------------------
X = XGrid(:);
Y = YGrid(:);
Z = ZGrid(:);

%------------------------------
% create numbering scheme
%------------------------------
Number.Phase      =   zeros(Ny + Ny-1, Nx + Nx-1);  % Create the general numbering scheme
Number_ind        =   zeros(Ny + Ny-1, Nx + Nx-1);  % Create the general numbering scheme
Number.PathY      =   zeros(Ny-1,Nx  );
Number.PathX      =   zeros(Ny  ,Nx-1);
Number.Center     =   zeros(Ny-1,Nx-1);
Number.Nodes     =   zeros(Ny,Nx);

for ix=1:2:Nx+Nx-1, for iz=2:2:Ny+Ny-1, Number.Phase(iz,ix) = 2; end; end % Paths in y-direction
for ix=2:2:Nx+Nx-1, for iz=1:2:Ny+Ny-1, Number.Phase(iz,ix) = 3; end; end % Paths in x-direction
for ix=2:2:Nx+Nx-1, for iz=2:2:Ny+Ny-1, Number.Phase(iz,ix) = 4; end; end % diagonal paths
for ix=1:2:Nx+Nx-1, for iz=1:2:Ny+Ny-1, Number.Phase(iz,ix) = 1; end; end % nodes

num               =     1;

% yPath numbering
for ix=1:size(Number.Phase,2)         % x horizontal direction
    for iz=1:size(Number.Phase,1)     % z vertical direction
        if Number.Phase(iz,ix)==2
            Number_ind(iz,ix)   =   num;    % Number_ind numbers all the variables (Vx,Vz,P) in the grid
            num                 =   num+1;
        end
    end
end

% xPath
for ix=1:size(Number.Phase,2)         % x horizontal direction
    for iz=1:size(Number.Phase,1)     % z vertical direction
        if Number.Phase(iz,ix)==3
            Number_ind(iz,ix)   =   num;    % Number_ind numbers all the variables (Vx,Vz,P) in the grid
            num                 =   num+1;
        end
    end
end

% %Center-numbering (not done to leave diagonal paths out)
% for ix=1:size(Number.Phase,2)         % x horizontal direction
%     for iz=1:size(Number.Phase,1)     % z vertical direction
%         if Number.Phase(iz,ix)==4
%             Number_ind(iz,ix)   =   num;    % Number_ind numbers all the variables (Vx,Vz,P) in the grid
%             num                 =   num+1;
%         end
%     end
% end

num = 1;

%corner point-numbering
for ix=1:size(Number.Phase,2)         % x horizontal direction
    for iz=1:size(Number.Phase,1)     % z vertical direction
        if Number.Phase(iz,ix)==1
            Number_ind(iz,ix)   =   num;    % Number_ind numbers all the variables (Vx,Vz,P) in the grid
            num                 =   num+1;
        end
    end
end

num_eqns         =   num-1;
% Now num_eqns is the total number of points of Vx, Vz and P, and thus number of equations
% Number of Vx = (Nz-1)*Nx
% Number of Vz = (Nx-1)*Nz
% Number of P  = (Nx-1)*(Nz-1)
% total number = 3*Nx*Nz-2*(Nx+nz)+1

% Find indices of Vx, Vz, and P in the general matrix
% ind_Vx, ind_Vz and ind_P are the order they appear in the matrix solution
ind_PathY    =   find(Number.Phase==2);  Number.PathY(find(Number.PathY==0)) = Number_ind(ind_PathY);
ind_PathX    =   find(Number.Phase==3);  Number.PathX(find(Number.PathX==0)) = Number_ind(ind_PathX);
ind_Center =   find(Number.Phase==4);  Number.Center(find(Number.Center ==0)) = Number_ind(ind_Center );
ind_Corner =   find(Number.Phase==1);  Number.Nodes(find(Number.Nodes ==0)) = Number_ind(ind_Corner );

% create a complete number grid with all numbers
Number.complete = zeros(size(Number.Phase));

Number.complete(ind_PathY) = Number.PathY;
Number.complete(ind_PathX) = Number.PathX;
Number.complete(ind_Center)  = Number.Center;

NumberNodes = zeros(size(Number.Phase));
NumberNodes(ind_Corner) = Number.Nodes;

% create vector that contains the node numbers per path (the row index represents the path)
PathVec = zeros(max(Number.complete(:)),2);

% paths in x-direction
for iPath = 1:length(Number.PathX(:));
    % 1. find adjacent nodes to paths
    [iz,ix]   = find(Number.complete==Number.PathX(iPath));
    PathVec(Number.PathX(iPath),1) = NumberNodes(iz,ix-1); % left
    PathVec(Number.PathX(iPath),2) = NumberNodes(iz,ix+1); % right   
end

% paths in y-direction
for iPath = 1:length(Number.PathY(:));
    % 1. find adjacent nodes to paths
    [iz,ix]   = find(Number.complete==Number.PathY(iPath));
    PathVec(Number.PathY(iPath),1) = NumberNodes(iz-1,ix); % left
    PathVec(Number.PathY(iPath),2) = NumberNodes(iz+1,ix); % right   
end

% diagonal paths (not used here, therefore we don't have to care about them)



%==========================================================================
% to get an undirected graph, flip the PathVec columns and add to PathVec
PathVec = [PathVec;fliplr(PathVec)];

%==========================================================================
% for each path, compute the topography gradient

% to do this, look at each path and compute the height difference between
% the nodes
% the Number.Nodes structure contains the numbers that can then be related
% to the coordinates. Since the 
% e.g.
% X(PathVec,1) gives the x-coordinates for the starting nodes
% X(PathVec,2) gives the x-coordinates for the ending nodes



Distance   =  sqrt((X(PathVec(:,2))-X(PathVec(:,1))).^2+(Y(PathVec(:,2))-Y(PathVec(:,1))).^2 + (Z(PathVec(:,2))-Z(PathVec(:,1))).^2); % compute distance between the nodes
DiffHeight =  Z(PathVec(:,2))-Z(PathVec(:,1)); % compute height difference between nodes, positive:upwards, negative: downwards 
Gradient   =  DiffHeight./Distance;

%==========================================================================
if Debug == 1
   figure(99),clf
   hold on
   for i = 1:size(PathVec,1),plot([X(PathVec(i,1)) X(PathVec(i,2))],[Y(PathVec(i,1)) Y(PathVec(i,2))],'k+-'),end  
end



