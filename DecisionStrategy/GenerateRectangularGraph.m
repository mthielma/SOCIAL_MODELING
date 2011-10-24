function [X,Y,Number,GCOORD,RegularElementNumber,ELEM2NODE,ElemArea] = GenerateRectangularGraph(xvec,yvec)
% generate graph for a rectangular net of streets. Paths can be added later

if nargin == 0
   xvec = 0:10;
   yvec = 0:10;
end
% INPUT:
% xvec, yvec: vectors that contain the coordinates of the physical grid

% OUTPUT:
% X,Y : structure that contains the x- or z- coordinates for the staggered
% grid, fields are: Qx, Qy, H_center, H_interp 
% 
% Number


Nx = length(xvec);
Ny = length(yvec);

%------------------------------
% create coordinate matrices
%------------------------------
yvec = yvec';

% Corner point grid
X.grid = repmat(xvec,Ny,1);
Y.grid = repmat(yvec,1,Nx);

%Qx-grid 
X.Qx	  = [X.grid(2:Ny,:)+X.grid(1:Ny-1,:)]/2;
Y.Qx	  = [Y.grid(2:Ny,:)+Y.grid(1:Ny-1,:)]/2;

%Vz-Grid
X.Qy	  = [X.grid(:,2:Nx)+X.grid(:,1:Nx-1)]/2;
Y.Qy	  = [Y.grid(:,2:Nx)+Y.grid(:,1:Nx-1)]/2;

%P-grid
X.Center	  = [X.grid(2:Ny,2:Nx)+X.grid(1:Ny-1,1:Nx-1)]/2;
Y.Center	  = [Y.grid(2:Ny,2:Nx)+Y.grid(1:Ny-1,1:Nx-1)]/2;


% area of elements
dx_elem = X.Qx(:,2:end)-X.Qx(:,1:end-1);
dz_elem = Y.Qy(2:end,:)-Y.Qy(1:end-1,:);

ElemArea = dx_elem.*dz_elem;
%------------------------------
% create numbering scheme
%------------------------------
Number.Phase      =   zeros(Ny + Ny-1, Nx + Nx-1);  % Create the general numbering scheme
Number_ind        =   zeros(Ny + Ny-1, Nx + Nx-1);  % Create the general numbering scheme
Number.PathY      =   zeros(Ny-1,Nx  );
Number.PathX      =   zeros(Ny  ,Nx-1);
Number.Center     =   zeros(Ny-1,Nx-1);
Number.Corner     =   zeros(Ny,Nx);

for ix=1:2:Nx+Nx-1, for iz=2:2:Ny+Ny-1, Number.Phase(iz,ix) = 2; end; end % Qx equations
for ix=2:2:Nx+Nx-1, for iz=1:2:Ny+Ny-1, Number.Phase(iz,ix) = 3; end; end % Qz equations
for ix=2:2:Nx+Nx-1, for iz=2:2:Ny+Ny-1, Number.Phase(iz,ix) = 4; end; end % Center equations
for ix=1:2:Nx+Nx-1, for iz=1:2:Ny+Ny-1, Number.Phase(iz,ix) = 1; end; end % Corner equations

num               =     1;

% Qx numbering
for ix=1:size(Number.Phase,2)         % x horizontal direction
    for iz=1:size(Number.Phase,1)     % z vertical direction
        if Number.Phase(iz,ix)==2
            Number_ind(iz,ix)   =   num;    % Number_ind numbers all the variables (Vx,Vz,P) in the grid
            num                 =   num+1;
        end
    end
end

% Qz numbering
for ix=1:size(Number.Phase,2)         % x horizontal direction
    for iz=1:size(Number.Phase,1)     % z vertical direction
        if Number.Phase(iz,ix)==3
            Number_ind(iz,ix)   =   num;    % Number_ind numbers all the variables (Vx,Vz,P) in the grid
            num                 =   num+1;
        end
    end
end

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

% %Center-numbering
% for ix=1:size(Number.Phase,2)         % x horizontal direction
%     for iz=1:size(Number.Phase,1)     % z vertical direction
%         if Number.Phase(iz,ix)==3
%             Number_ind(iz,ix)   =   num;    % Number_ind numbers all the variables (Vx,Vz,P) in the grid
%             num                 =   num+1;
%         end
%     end
% end



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
ind_Corner =   find(Number.Phase==1);  Number.Corner(find(Number.Corner ==0)) = Number_ind(ind_Corner );

% create a complete grid with all coordinates
X.complete = zeros(size(Number.Phase));
Y.complete = zeros(size(Number.Phase));

X.complete(ind_PathY) = X.Qx;
Y.complete(ind_PathY) = Y.Qx;

X.complete(ind_PathX) = X.Qy;
Y.complete(ind_PathX) = Y.Qy;

X.complete(ind_Center)  = X.Center;
Y.complete(ind_Center)  = Y.Center;

X.complete(ind_Corner) = X.grid;
Y.complete(ind_Corner) = Y.grid;

% create a complete number grid with all numbers
Number.complete = zeros(size(Number.Phase));
Number.complete = zeros(size(Number.Phase));

Number.complete(ind_PathY) = Number.PathY;
Number.complete(ind_PathY) = Number.PathY;

Number.complete(ind_PathX) = Number.PathX;
Number.complete(ind_PathX) = Number.PathX;

Number.complete(ind_Center)  = Number.Center;
Number.complete(ind_Center)  = Number.Center;

%Number.complete(ind_Corner) = Number.Corner;
%Number.complete(ind_Corner) = Number.Corner;


NumberNodes = zeros(size(Number.Phase));
NumberNodes(ind_Corner) = Number.Corner;
% create vector that contains the node numbers per path (the row index represents the path)
PathVec = zeros(max(Number.complete(:)),2);

% paths in x-direction
for iPath = 1:length(Number.PathX(:));
    % 1. find adjacent nodes to paths
    [iz,ix]   = find(Number.complete==Number.PathX(iPath));
    PathVec(Number.PathX(iPath),1) = NumberNodes(iz,ix-1); % left
    PathVec(Number.PathX(iPath),2) = NumberNodes(iz,ix+1); % right   
end

% paths in z-direction
for iPath = 1:length(Number.PathY(:));
    % 1. find adjacent nodes to paths
    [iz,ix]   = find(Number.complete==Number.PathY(iPath));
    PathVec(Number.PathY(iPath),1) = NumberNodes(iz-1,ix); % left
    PathVec(Number.PathY(iPath),2) = NumberNodes(iz+1,ix); % right   
end


Number.complete(ind_Corner) = Number.Corner;
Number.complete(ind_Corner) = Number.Corner;


bla;



