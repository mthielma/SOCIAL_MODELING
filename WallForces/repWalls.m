

% repulsive walls
% fabiocrameri 2011.1026

clear
display('*******')
%---------------------------------------------
%input ---------------------------------------

runName     = 'test';
saveMovie   = logical(0);

nx          = 100;
ny          = 100;

attrSpread  = {'exp' 'linear' 'const'};
attrSpread  = attrSpread{3};

attrForce   = 0.2; %1 is the same as wall force

%Architecture position
% X-POSITION(left right) / Y-POSITION(bottom top) / TYPE(1:repulsive 2:attractor)
Arch = [
    10 30       30 35   1
    35 40       36 40   2
    30 34       15 20   1
    18 20       10 25 	1
    ...%26 27       28 30   1
];



%---------------------------------------------
%describe wall -------------------------------
display('describing walls')

GRIDwall = zeros(nx,ny);
GRIDattr = zeros(nx,ny);
grid = zeros(nx,ny,2);

for i=1:nx
    for j=1:ny
        
        grid(i,j,:) = [i j];
        
        %loop all architecture
        for k=1:size(Arch,1)
            if ( i>=Arch(k,1) && i<=Arch(k,2)...
                    && j>=Arch(k,3) && j<=Arch(k,4) )
                if (Arch(k,5)==1)
                    GRIDwall(i,j) = 1;  %is wall
                elseif (Arch(k,5)==2)
                    GRIDattr(i,j) = -1; %is attractor
                end
            end
        end
        
    end
end
GRIDarch = GRIDwall+GRIDattr;


figure(1),clf
subplot(3,3,1)
imagesc(GRIDarch')
title('walls')
xlabel('x')
ylabel('y')
colorbar
axis equal; axis tight


%---------------------------------------------
%distance to wall ----------------------------
display('calculating distance to architecture and architecture force')

distToWall = ones(nx,ny)*nx*ny; %make sure it is big enough
distToAttr = ones(nx,ny)*nx*ny; %make sure it is big enough
F_wall = zeros(nx,ny);
F_attr = zeros(nx,ny);
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
if strcmp(attrSpread,'exp')
    F_attr = -exp(-distToAttr);
elseif strcmp(attrSpread,'linear')    
    F_attr = attrForce*(-1+distToAttr./max(max(distToAttr)));
elseif strcmp(attrSpread,'const')
    F_attr = -attrForce;
end

F_arch = F_wall+F_attr;


subplot(3,3,3)
imagesc(distToWall')
title('distance to wall')
xlabel('x')
ylabel('y')
colorbar
axis equal; axis tight

subplot(3,3,2)
imagesc(distToAttr')
title('distance to attractor')
xlabel('x')
ylabel('y')
colorbar
axis equal; axis tight

subplot(3,3,4)
imagesc(F_arch')
title('architecture force magnitude')
xlabel('x')
ylabel('y')
colorbar
axis equal; axis tight


%---------------------------------------------
%force direction -----------------------------
display('calculating force direction')

xgradWall = zeros(nx,ny); xgradAttr = zeros(nx,ny);
ygradWall = zeros(nx,ny); ygradAttr = zeros(nx,ny);
for i=2:nx-1
    for j=2:ny-1
        
        xgradWall(i,j) = distToWall(i+1,j)-distToWall(i-1,j);
        ygradWall(i,j) = distToWall(i,j+1)-distToWall(i,j-1);
        
        xgradAttr(i,j) = -(distToAttr(i+1,j)-distToAttr(i-1,j));
        ygradAttr(i,j) = -(distToAttr(i,j+1)-distToAttr(i,j-1));
        
    end
end

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


subplot(3,3,6)
quiver(grid(:,:,1),grid(:,:,2),xgradWall,ygradWall)
axis ij
title('wall force direction')
xlabel('x')
ylabel('y')
axis equal; axis tight

subplot(3,3,5)
quiver(grid(:,:,1),grid(:,:,2),xgradAttr,ygradAttr)
axis ij
title('attractors force direction')
xlabel('x')
ylabel('y')
axis equal; axis tight

subplot(3,3,9)
quiver(grid(:,:,1),grid(:,:,2),xgradF_wall,ygradF_wall)
axis ij
title('wall force')
xlabel('x')
ylabel('y')
axis equal; axis tight

subplot(3,3,8)
quiver(grid(:,:,1),grid(:,:,2),xgradF_attr,ygradF_attr)
axis ij
title('attractor force')
xlabel('x')
ylabel('y')
axis equal; axis tight

subplot(3,3,7)
quiver(grid(:,:,1),grid(:,:,2),xgradF_arch,ygradF_arch)
axis ij
title('architecture force')
xlabel('x')
ylabel('y')
axis equal; axis tight






%let's move!
t_max = 1000;


%initial position
Don = [ 5 5 ];  %x y
DonPath = zeros(t_max,2).*NaN;

for t=1:t_max
    DonPath(t,:) = Don;
    
    Don = Don + [xgradF_arch(round(Don(1,1)),round(Don(1,2))) ygradF_arch(round(Don(1,1)),round(Don(1,2)))];
    
       
    figure(2)
    imagesc(GRIDarch')
    hold on
    plot(Don(1,1),Don(1,2),'ok','MarkerFaceColor','k')
    
    plot(DonPath(:,1),DonPath(:,2),'k')
    axis ij
    
    
    
    
    
    Mov(t) = getframe; %save frames for movie
    
    
    
    if ( Don(1,1)>nx-5 && Don(1,2)>ny-5 )
        break
    end
end


movie(Mov)  %show movie

movie2avi(Mov,[runName]);  %save movie
display(['Movie ',runName,'.avi was saved to ',pwd])




















