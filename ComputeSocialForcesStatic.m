function [FxSocialStatic,FySocialStatic] = ComputeSocialForcesStatic(AGENT,X_Grid,Y_Grid,xArchForces,yArchForces,Parameter)

FxSocialStatic = interp2(X_Grid,Y_Grid,xArchForces',[AGENT.LocX],[AGENT.LocY],'*linear').*exp([AGENT.Size]./Parameter.B);
FySocialStatic = interp2(X_Grid,Y_Grid,yArchForces',[AGENT.LocX],[AGENT.LocY],'*linear').*exp([AGENT.Size]./Parameter.B);
