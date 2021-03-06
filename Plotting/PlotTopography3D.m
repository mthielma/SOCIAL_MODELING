function PlotTopography3D(X_Grid,Y_Grid,Z_Grid)

display('...plotting topography')

    hs2=surfl(X_Grid,Y_Grid,Z_Grid); set(hs2,'EdgeColor','none');
    colormap('gray')
    
    set(hs2,'FaceLighting','phong','AmbientStrength',0.3,'DiffuseStrength',0.8,...
        'SpecularStrength',0.9,'SpecularExponent',25,'BackFaceLighting','lit');
    
    hold on
    contour3(X_Grid,Y_Grid,Z_Grid,'k--')
    hold on