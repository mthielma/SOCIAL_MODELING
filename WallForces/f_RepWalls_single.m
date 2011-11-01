
%=============================
% 
% function f_RepWalls_single.m
% 
%=============================
% Fabio Crameri 2011-11-01

% % EXAMPLE INPUT:
% *******************
% nx = 40;
% ny = 40;
% % Architecture position
% % X-POSITION(left right) / Y-POSITION(bottom top)
% Arch = [ 10 30     30 35 ];  or   size(nx+1,ny+1)
% Arch = 'list'                or 	'map'
% Type = 1; %1: repulsive / 2: attractive
% Spreading  = {'exp' 'linear' 'const'};
% Spreading  = Spreading{3};
% Force   = 0.2; %1 is the same as wall force
% 
% [xArchForces, yArchForces, grid] = f_RepWalls_single (nx, ny, Arch, ArchFormat, Type, Spreading, Force)
% ********************

function [xArchForces, yArchForces, grid] = f_RepWalls_single (nx, ny, Arch, ArchFormat, Type, Spreading, Force)
plotFields = logical(0);

display('******* calculate wall forces ******')

%---------------------------------------------
%describe wall -------------------------------
display('describing walls')

grid = zeros(nx,ny,2);

if strcmp(ArchFormat,'list')
    GRIDwall = zeros(nx,ny);
    GRIDattr = zeros(nx,ny);
    for i=1:nx
        for j=1:ny
            grid(i,j,:) = [i j];
            
            %loop all architecture
            for k=1:size(Arch,1)
                if ( i>=Arch(k,1) && i<=Arch(k,2)...
                        && j>=Arch(k,3) && j<=Arch(k,4) )
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
    for i=1:nx
        for j=1:ny
            grid(i,j,:) = [i j];
        end
    end
   GRIDarch = Arch';
   
else
    error('Hei stupid: insert correct ArchFormat!')
end

if plotFields
    figure(1),clf
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

distToWall = ones(nx,ny)*nx*ny; %make sure it is big enough
distToAttr = ones(nx,ny)*nx*ny; %make sure it is big enough
for i=1:nx
    for j=1:ny
        %find distance to wall
        %loop architecture
        for ii=1:nx
            for jj=1:ny
                if ( sqrt((ii-i)^2+(jj-j)^2)<distToWall(i,j) && GRIDarch(ii,jj)==1 )  %get shortest distance to walls
                    distToWall(i,j) = sqrt( (ii-i)^2 + (jj-j)^2 );
                end
                if( sqrt((ii-i)^2+(jj-j)^2)<distToAttr(i,j) && GRIDarch(ii,jj)==-1 )  %get shortest distance to attractors
                    distToAttr(i,j) = sqrt( (ii-i)^2 + (jj-j)^2 );
                end
            end
        end
    end
end

%---------------------------------------------
%wall force ----------------------------------
% f =  %from Helbing2000
F_wall = exp(-distToWall);    %max. force value should be 1.0
if strcmp(Spreading,'exp')
    F_attr = -exp(-distToAttr);
elseif strcmp(Spreading,'linear')    
    F_attr = Force*(-1+distToAttr./max(max(distToAttr)));
elseif strcmp(Spreading,'const')
    F_attr = -Force;
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
% for i=2:nx-1
%     for j=2:ny-1
%         xgradWall(i,j) = distToWall(i+1,j)-distToWall(i-1,j);
%         ygradWall(i,j) = distToWall(i,j+1)-distToWall(i,j-1);
%         
%         xgradAttr(i,j) = -(distToAttr(i+1,j)-distToAttr(i-1,j));
%         ygradAttr(i,j) = -(distToAttr(i,j+1)-distToAttr(i,j-1));
%     end
% end

xgradWall(2:nx-1,2:ny-1) = distToWall(3:nx,2:ny-1) - distToWall(1:nx-2,2:ny-1);
ygradWall(2:nx-1,2:ny-1) = distToWall(2:nx-1,3:ny) - distToWall(2:nx-1,1:ny-2);

xgradAttr(2:nx-1,2:ny-1) = -( distToAttr(3:nx,2:ny-1) - distToAttr(1:nx-2,2:ny-1) );
ygradAttr(2:nx-1,2:ny-1) = -( distToAttr(2:nx-1,3:ny) - distToAttr(2:nx-1,1:ny-2) );


%boundary conditions -----------------------------
xgradWall(1,:)  = 0;   ygradWall(1,:)  = 0; %open boundaries
xgradWall(nx,:) = 0;   ygradWall(nx,:) = 0;
xgradWall(:,1)  = 0;   ygradWall(:,1)  = 0;
xgradWall(:,ny) = 0;   ygradWall(:,ny) = 0;

xgradAttr(1,:)  = 0;   ygradAttr(1,:)  = 0; %open boundaries
xgradAttr(nx,:) = 0;   ygradAttr(nx,:) = 0;
xgradAttr(:,1)  = 0;   ygradAttr(:,1)  = 0;
xgradAttr(:,ny) = 0;   ygradAttr(:,ny) = 0;
% -------------------------------------------------

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
    quiver(grid(:,:,1),grid(:,:,2),xgradWall,ygradWall)
    axis ij
    title('wall force direction')
    xlabel('x')
    ylabel('y')
    axis equal; axis tight, axis ij
    
    subplot(3,3,5)
    quiver(grid(:,:,1),grid(:,:,2),xgradAttr,ygradAttr)
    axis ij
    title('attractors force direction')
    xlabel('x')
    ylabel('y')
    axis equal; axis tight, axis ij
    
    subplot(3,3,9)
    quiver(grid(:,:,1),grid(:,:,2),xgradF_wall,ygradF_wall)
    axis ij
    title('wall force')
    xlabel('x')
    ylabel('y')
    axis equal; axis tight, axis ij
    
    subplot(3,3,8)
    quiver(grid(:,:,1),grid(:,:,2),xgradF_attr,ygradF_attr)
    axis ij
    title('attractor force')
    xlabel('x')
    ylabel('y')
    axis equal; axis tight, axis ij
    
    subplot(3,3,7)
    quiver(grid(:,:,1),grid(:,:,2),xgradF_arch,ygradF_arch)
    axis ij
    title('architecture force')
    xlabel('x')
    ylabel('y')
    axis equal; axis tight, axis ij
end

xArchForces = xgradF_arch;
yArchForces = ygradF_arch;



