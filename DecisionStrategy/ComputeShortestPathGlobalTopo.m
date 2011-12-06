function [Dgradx_topo,Dgrady_topo,D_topo] = ComputeShortestPathGlobalTopo(BuildingMap,ExitMap,X_Grid,Y_Grid,Z_Grid,D_orig,TopoGradientX,TopoGradientY,Parameter)

Debug = false;

tol = 1e-3;
%==============================================================
% FIRST GUESS
%==============================================================
F_topo = ones(size(X_Grid))*Parameter.v0;
% add buildings to map
% add buildings to map
F_topo(BuildingMap) = 1e-8;
% find indices of exits
[indx,indy] = find(ExitMap == 1);

ExitPoints = [indx';indy'];
% use fast marching algorithm to compute distance to exit
[D_topo]= msfm(F_topo, ExitPoints);

% D_topo  = D_topo-D_orig;
% D_topo = D_topo./max(D_topo(~BuildingMap)); % scale to max (except building values)
% compute gradient in time -> this gives us the direction
[Dgradx_orig,Dgrady_orig] = gradient(D_topo,Parameter.resolution,Parameter.resolution);

% scale direction vectors to normal
Dgradtot = sqrt(Dgradx_orig.^2+Dgrady_orig.^2);
Dgradx_topo   = -Dgradx_orig./Dgradtot;
Dgrady_topo   = -Dgrady_orig./Dgradtot;

if Debug
    figure(99),clf
    hold on
    pcolor(X_Grid,Y_Grid,Z_Grid), shading flat
    quiver(X_Grid,Y_Grid,Dgradx_topo,Dgrady_topo)
    axis equal, axis tight
end



if var(Z_Grid(:))>1e-2;
    for i = 1:10
        
        %==============================================================
        % RECOMPUTE WITH TOPOGRAPHY
        %==============================================================
        % recompute the velocity field based on the slope
        slope = TopoGradientX.*Dgradx_topo+TopoGradientY.*Dgrady_topo;
        
        % compute maximum velocity due to slope
        PreFac = (Parameter.v0./exp(-3.5*0.05));
        F_topo = PreFac.*exp(-3.5.*abs(slope+0.05));
        
        % add buildings to map
        F_topo(BuildingMap) = 1e-8;
        
        [D_topo]=msfm(F_topo, ExitPoints);
        alpha = 0.7;
        D_Topo = ((1-alpha).*D_orig + alpha.*D_topo);
        [Dgradx_topo,Dgrady_topo] = gradient(D_topo,Parameter.resolution,Parameter.resolution);
        % scale direction vectors to normal
        Dgradtot = sqrt(Dgradx_topo.^2+Dgrady_topo.^2);
        Dgradx_topo   = -Dgradx_topo./Dgradtot;
        Dgrady_topo   = -Dgrady_topo./Dgradtot;
        
        
        err_val = norm(D_topo-D_orig);
        D_orig = D_topo;
        
        if Debug
            figure(99),clf
            hold on
            pcolor(X_Grid,Y_Grid,Z_Grid), shading flat
            startx = zeros(17,1);
            starty = 1:0.5:9;
            contour(X_Grid,Y_Grid,slope)
            streamline(X_Grid,Y_Grid,Dgradx_topo,Dgrady_topo,startx,starty)
            %quiver(X_Grid,Y_Grid,Dgradx_topo,Dgrady_topo,'w')
            title([num2str(err_val),num2str(max(slope(:)))])
            axis equal, axis tight
        end
    end
end
bla = 1;

