function PlotTopography3D(X_Grid,Y_Grid,Z_Grid)
    hs=surfl(X_Grid,Y_Grid,Z_Grid); set(hs,'EdgeColor','none');
    colormap('gray')
    
    set(hs,'FaceLighting','phong','AmbientStrength',0.3,'DiffuseStrength',0.8,...
        'SpecularStrength',0.9,'SpecularExponent',25,'BackFaceLighting','lit');
    
    
    contour3(X_Grid,Y_Grid,Z_Grid,'k--')