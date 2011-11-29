
%=============================
% 
% function f_RepWalls_single.m
% 
%=============================
% Fabio Crameri 2011-11-02

% % EXAMPLE INPUT:
% *******************
% % Architecture position
% % X-POSITION(left right) / Y-POSITION(bottom top)
% ARCH.geometry     = [ 10 30     30 35 ];  or   size(nx+1,ny+1)
% ARCH.format       = 'list'                or 	'map'
% ARCH.type         = 1; %1: repulsive / 2: attractive
% Spreading         = {'exp' 'linear' 'const'};
% ARCH.spreading    = Spreading{3};
% ARCH.force        = 0.2; %1 is the same as wall force
% 
% [xArchForces, yArchForces, xArchDir, yArchDir] = f_RepWalls_single (X_Grid, Y_Grid, ArchGeometry, ARCH, Parameter)
% ********************

function [F_arch, xArchForces, yArchForces, xArchDir, yArchDir] = f_RepWalls_single (X_Grid, Y_Grid, ArchGeometry, ARCH, Parameter)
plotFields = logical(0);


X_Grid  = X_Grid';  %convert grid
Y_Grid  = Y_Grid';

Arch        = ArchGeometry;
ArchFormat  = ARCH.format;
Type        = ARCH.type;
Spreading   = ARCH.spreading;
Force       = ARCH.force;

A           = Parameter.A;
B           = Parameter.B;
ExitFactor  = Parameter.ExitFactor;

display('******* calculate wall forces ******')

%---------------------------------------------
%describe wall -------------------------------
display('describing walls')

nx   	= size(X_Grid,1);
ny   	= size(Y_Grid,2);

if strcmp(ArchFormat,'list')
    GRIDwall = zeros(nx,ny);
    GRIDattr = zeros(nx,ny);
    for i=1:nx
        for j=1:ny
            %loop all architecture
            for k=1:size(Arch,1)
                if ( X_Grid(i,j)>=Arch(k,1) && X_Grid(i,j)<=Arch(k,2)...
                        && Y_Grid(i,j)>=Arch(k,3) && Y_Grid(i,j)<=Arch(k,4) )
                    if (Type==1)
                        GRIDwall(i,j) = 1;  %is wall
                    elseif (Type==2)
                        GRIDattr(i,j) = -1; %is attractor
                    end
                end
            end
        end
    end
    GRIDarch = GRIDwall+GRIDattr;
    
elseif strcmp(ArchFormat,'map')
   GRIDarch = Arch';
else
    error('Hei stupid: insert correct ArchFormat!')
end

% %Domain Boundaries
% GRIDarch(1,:) = 3;  GRIDarch(nx,:) = 3;
% GRIDarch(:,1) = 3;  GRIDarch(:,ny) = 3;

if plotFields
    figure(12),clf
    subplot(3,3,1)
    imagesc(GRIDarch')
    title('walls')
    xlabel('x')
    ylabel('y')
    colorbar
    axis equal; axis tight
end

%---------------------------------------------
%distance to wall ----------------------------
display('calculating distance to architecture and architecture force')

X_Grid_Wall = X_Grid(GRIDarch==1);  X_Grid_Attr = X_Grid(GRIDarch==-1);
Y_Grid_Wall = Y_Grid(GRIDarch==1);  Y_Grid_Attr = Y_Grid(GRIDarch==-1);

distToWall = ones(nx,ny)*max(max(X_Grid))*max(max(Y_Grid)); %make sure it is big enough
distToAttr = distToWall; %make sure it is big enough

%find distance to architecture of each grid point
%loop only through architecture
for ii=1:size(X_Grid_Wall,1)
    distToWall = min( distToWall, sqrt( (X_Grid-X_Grid_Wall(ii)).^2 + (Y_Grid-Y_Grid_Wall(ii)).^2 )  );
end
for ii=1:size(X_Grid_Attr,1)
    distToAttr = min( distToAttr, sqrt( (X_Grid-X_Grid_Attr(ii)).^2 + (Y_Grid-Y_Grid_Attr(ii)).^2 )  );
end



%---------------------------------------------
%arch force ----------------------------------
if strcmp(Spreading,'exp')
    %this is only one part of: A*exp[(r-d)/B]*n
    %--------------------------------------------------------
    F_wall = + A .* exp(-distToWall./B);	%from Helbing2000
    F_attr = - A .* exp(-distToAttr./B);  
    %--------------------------------------------------------
elseif strcmp(Spreading,'linear')    
    %--------------------------------------------------------
    F_wall = Force.*(+1-distToWall./max(max(distToWall)));  %check!
    F_attr = Force.*(-1+distToAttr./max(max(distToAttr)));
    %--------------------------------------------------------
elseif strcmp(Spreading,'const')
    %--------------------------------------------------------
    F_wall = ExitFactor*A* (+Force);  %check!
    F_attr = ExitFactor*A* (-Force);
    %--------------------------------------------------------
end
F_arch = F_wall+F_attr;

if plotFields
    subplot(3,3,3)
    imagesc(distToWall')
    title('distance to wall')
    xlabel('x')
    ylabel('y')
    colorbar
    axis equal; axis tight, axis ij
    
    subplot(3,3,2)
    imagesc(distToAttr')
    title('distance to attractor')
    xlabel('x')
    ylabel('y')
    colorbar
    axis equal; axis tight, axis ij
    
    subplot(3,3,4)
    imagesc(F_arch')
    title('architecture force magnitude')
    xlabel('x')
    ylabel('y')
    colorbar
    axis equal; axis tight, axis ij
end

%---------------------------------------------
%force direction -----------------------------
display('calculating force direction')

xgradWall = zeros(nx,ny); xgradAttr = zeros(nx,ny);
ygradWall = zeros(nx,ny); ygradAttr = zeros(nx,ny);
%--
xgradWall(2:nx-1,2:ny-1) = distToWall(3:nx,2:ny-1) - distToWall(1:nx-2,2:ny-1);
ygradWall(2:nx-1,2:ny-1) = distToWall(2:nx-1,3:ny) - distToWall(2:nx-1,1:ny-2);

xgradWall2  = xgradWall./sqrt(xgradWall.^2+ygradWall.^2);   %normalized to 1
ygradWall2  = ygradWall./sqrt(xgradWall.^2+ygradWall.^2);   %normalized to 1
xgradWall   = xgradWall2; xgradWall(isnan(xgradWall))=0;    %replace NaN values with 0 
ygradWall   = ygradWall2; ygradWall(isnan(ygradWall))=0;    %replace NaN values with 0
%--
xgradAttr(2:nx-1,2:ny-1) = -( distToAttr(3:nx,2:ny-1) - distToAttr(1:nx-2,2:ny-1) );
ygradAttr(2:nx-1,2:ny-1) = -( distToAttr(2:nx-1,3:ny) - distToAttr(2:nx-1,1:ny-2) );

xgradAttr2  = xgradAttr./sqrt(xgradAttr.^2+ygradAttr.^2);   %normalized to 1
ygradAttr2  = ygradAttr./sqrt(xgradAttr.^2+ygradAttr.^2);   %normalized to 1
xgradAttr   = xgradAttr2; xgradAttr(isnan(xgradAttr))=0;    %replace NaN values with 0
ygradAttr   = ygradAttr2; ygradAttr(isnan(ygradAttr))=0;    %replace NaN values with 0
%--

% -------------------------------------------------
%BOUNDARIES
% xgradWall(1,:)      = distToWall(2,:)       - distToWall(1,:);
% xgradWall(nx,:)     = distToWall(nx,:)      - distToWall(nx-1,:);
% xgradWall(2:nx-1,1) = distToWall(2:nx-1,2)  - distToWall(2:nx-1,1);
% xgradWall(2:nx-1,ny)= distToWall(2:nx-1,ny) - distToWall(2:nx-1,ny-1);
% ygradWall(:,1)      = distToWall(:,2)       - distToWall(:,1);
% ygradWall(:,ny)     = distToWall(:,ny)      - distToWall(:,ny-1);
% ygradWall(1,2:ny-1) = distToWall(2,2:ny-1)  - distToWall(1,2:ny-1);
% ygradWall(nx,2:ny-1)= distToWall(nx,2:ny-1) - distToWall(nx-1,2:ny-1);
% 
% xgradAttr(1,:)      = distToAttr(2,:)       - distToAttr(1,:);
% xgradAttr(nx,:)     = distToAttr(nx,:)      - distToAttr(nx-1,:);
% xgradAttr(2:nx-1,1) = distToAttr(2:nx-1,2)  - distToAttr(2:nx-1,1);
% xgradAttr(2:nx-1,ny)= distToAttr(2:nx-1,ny) - distToAttr(2:nx-1,ny-1);
% ygradAttr(:,1)      = distToAttr(:,2)       - distToAttr(:,1);
% ygradAttr(:,ny)     = distToAttr(:,ny)      - distToAttr(:,ny-1);
% ygradAttr(1,2:ny-1) = distToAttr(2,2:ny-1)  - distToAttr(1,2:ny-1);
% ygradAttr(nx,2:ny-1)= distToAttr(nx,2:ny-1) - distToAttr(nx-1,2:ny-1);
% -------------------------------------------------



%boundary conditions -----------------------------
% xgradWall(1,:)  = 0;   ygradWall(1,:)  = 0; %open boundaries
% xgradWall(nx,:) = 0;   ygradWall(nx,:) = 0;
% xgradWall(:,1)  = 0;   ygradWall(:,1)  = 0;
% xgradWall(:,ny) = 0;   ygradWall(:,ny) = 0;
% 
% xgradAttr(1,:)  = 0;   ygradAttr(1,:)  = 0; %open boundaries
% xgradAttr(nx,:) = 0;   ygradAttr(nx,:) = 0;
% xgradAttr(:,1)  = 0;   ygradAttr(:,1)  = 0;
% xgradAttr(:,ny) = 0;   ygradAttr(:,ny) = 0;
% -------------------------------------------------
%boundary conditions -----------------------------
xgradWall(1,:)  = xgradWall(2,:);       ygradWall(1,:)  = ygradWall(2,:); %constant boundaries
xgradWall(nx,:) = xgradWall(nx-1,:);    ygradWall(nx,:) = ygradWall(nx-1,:);
xgradWall(:,1)  = xgradWall(:,2);       ygradWall(:,1)  = ygradWall(:,2);
xgradWall(:,ny) = xgradWall(:,ny-1);    ygradWall(:,ny) = ygradWall(:,ny-1);

xgradAttr(1,:)  = xgradAttr(2,:);       ygradAttr(1,:)  = ygradAttr(2,:); %constant boundaries
xgradAttr(nx,:) = xgradAttr(nx-1,:);    ygradAttr(nx,:) = ygradAttr(nx-1,:);
xgradAttr(:,1)  = xgradAttr(:,2);       ygradAttr(:,1)  = ygradAttr(:,2);
xgradAttr(:,ny) = xgradAttr(:,ny-1);    ygradAttr(:,ny) = ygradAttr(:,ny-1);
% -------------------------------------------------

%combine both:
xgrad_arch = xgradWall + xgradAttr;
ygrad_arch = ygradWall + ygradAttr;

%---------------------------------------------
%wall force direction  -----------------------
display('calculating architecture force direction')

xgradF_wall = xgradWall .* F_wall;
ygradF_wall = ygradWall .* F_wall;
xgradF_attr = -xgradAttr .* F_attr;
ygradF_attr = -ygradAttr .* F_attr;

%combine both:
xgradF_arch = xgradF_wall + xgradF_attr;
ygradF_arch = ygradF_wall + ygradF_attr;

if plotFields
    subplot(3,3,6)
    quiver(X_Grid,Y_Grid,xgradWall,ygradWall)
    axis ij
    title('wall force direction')
    xlabel('x')
    ylabel('y')
    axis equal; axis tight, axis ij
    
    subplot(3,3,5)
    quiver(X_Grid,Y_Grid,xgradAttr,ygradAttr)
    axis ij
    title('attractors force direction')
    xlabel('x')
    ylabel('y')
    axis equal; axis tight, axis ij
    
    subplot(3,3,9)
    quiver(X_Grid,Y_Grid,xgradF_wall,ygradF_wall)
    axis ij
    title('wall force')
    xlabel('x')
    ylabel('y')
    axis equal; axis tight, axis ij
    
    subplot(3,3,8)
    quiver(X_Grid,Y_Grid,xgradF_attr,ygradF_attr)
    axis ij
    title('attractor force')
    xlabel('x')
    ylabel('y')
    axis equal; axis tight, axis ij
    
    subplot(3,3,7)
    quiver(X_Grid,Y_Grid,xgradF_arch,ygradF_arch)
    axis ij
    title('architecture force')
    xlabel('x')
    ylabel('y')
    axis equal; axis tight, axis ij
end

xArchDir = xgrad_arch; %direction field normalized to 1 (xArchDir^2+yArchDir^2 = 1)
yArchDir = ygrad_arch;

xArchForces = xgradF_arch; %force field dimensional [N]
yArchForces = ygradF_arch;

F_arch = F_arch;  %Arch forces without direction



