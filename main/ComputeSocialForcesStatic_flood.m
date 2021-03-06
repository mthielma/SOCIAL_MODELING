function [FxSocialStatic,FySocialStatic] = ComputeSocialForcesStatic_flood(AGENT,X_Grid,Y_Grid,FloodForce,FloodDirX,FloodDirY,Parameter)

% compute total force taking into account agent size
ArchForceStatic = interp2(X_Grid,Y_Grid,FloodForce,[AGENT.LocX],[AGENT.LocY],'*linear').*exp([AGENT.Size]./Parameter.B_flood); 

% compute directional force
DirX = interp2(X_Grid,Y_Grid,FloodDirX,[AGENT.LocX],[AGENT.LocY],'*linear');
DirY = interp2(X_Grid,Y_Grid,FloodDirY,[AGENT.LocX],[AGENT.LocY],'*linear');

% norm direction vector
DirTot = sqrt(DirX.*DirX+DirY.*DirY);
DirX   = DirX./DirTot;
DirY   = DirY./DirTot;

% compute forces
FxSocialStatic = ArchForceStatic.*DirX;
FySocialStatic = ArchForceStatic.*DirY;