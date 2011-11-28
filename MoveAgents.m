function [AGENT] = MoveAgents(AGENT,X_Grid,Y_Grid,Gradient_x,Gradient_y,dt,nagent)

% according to [Helbing 2000]:
% a = dvi/dt = (v0_x - [AGENT.VelX])./t_acc + [AGENT.FxArch]./m + [AGENT.FxPedestrians]./m

t_acc = AGENT.t_acc;
m     = AGENT.m;

%==========================================================
% compute maximum desired velocity due to topography
%==========================================================

% interpolate topography gradient to agents
agent_gx = interp2(X_Grid,Y_Grid,Gradient_x,[AGENT.LocX],[AGENT.LocY],'*linear');
agent_gy = interp2(X_Grid,Y_Grid,Gradient_y,[AGENT.LocX],[AGENT.LocY],'*linear');
% compute slope in walking direction
slope = sum([agent_gx' agent_gy'].*[[AGENT(:).DirX]' [AGENT(:).DirY]'],2);
% limit maxmimum velocity
PreFac = ([AGENT(1:nagent).VMax]'./exp(-3.5*0.05));
V_max_agent = PreFac.*exp(-3.5.*abs(slope+0.05));

%==========================================================
% compute velocity change
%==========================================================

% velocity change in x-direction
v0_x                    = V_max_agent .* [AGENT(1:nagent).xExitDir]';
dvi_x                   = ( (v0_x - [AGENT.VelX]')./t_acc + [AGENT.FxSocialWalls]'./m + [AGENT.FxSocialAgents]'./m + [AGENT.FxPhysAgents]'./m + [AGENT.FxPhysWall]'./m + [AGENT.xForceExit]'./m) .*dt;	%change of velocity
v_x                     = v0_x + dvi_x;

% velocity change in y-direction
v0_y                    = V_max_agent .* [AGENT(1:nagent).yExitDir]';
dvi_y                   = ( (v0_y - [AGENT.VelY]')./t_acc + [AGENT.FySocialWalls]'./m + [AGENT.FySocialAgents]'./m + [AGENT.FyPhysAgents]'./m + [AGENT.FyPhysWall]'./m + [AGENT.yForceExit]'./m) .*dt;	%change of velocity
v_y                     = v0_y + dvi_y;

% compute total velocity and direction of agent
v_tot                   = sqrt(v_x.^2+v_y.^2);
dir_x                   = v_x./v_tot;
dir_y                   = v_y./v_tot;

%==========================================================
% recompute maximal velocity due to topgraphy
%==========================================================
% compute slope in walking direction
slope = sum([agent_gx' agent_gy'].*[dir_x dir_y],2);
% limit maxmimum velocity
PreFac = ([AGENT(1:nagent).VMax]'./exp(-3.5*0.05));
V_max_agent = PreFac.*exp(-3.5.*abs(slope+0.05));

%==========================================================
% limit velocity to maximum velocity
%==========================================================
v_tot                   = [min(v_tot,V_max_agent)];
dummy                   = num2cell(v_tot);
[AGENT(1:nagent).Vel]   = dummy{:};

%==========================================================
% recompute vx and vz
%==========================================================
dummy                   = num2cell(v_tot.*dir_x);
[AGENT(1:nagent).VelX]  = dummy{:};
dummy                   = num2cell(v_tot.*dir_y);
[AGENT(1:nagent).VelY]  = dummy{:};

%==========================================================
% update locations
%==========================================================
dummy                   = num2cell([AGENT.LocX] + [AGENT.VelX].*dt);
[AGENT(1:nagent).LocX]  = dummy{:};                                                     %update x-position

dummy                   = num2cell([AGENT.LocY] + [AGENT.VelY].*dt);
[AGENT(1:nagent).LocY]  = dummy{:};                                                     %update y-position

%==========================================================
% update direction
%==========================================================
dummy = num2cell(dir_x);
[AGENT(1:nagent).DirX]  = dummy{:};
dummy = num2cell(dir_y);
[AGENT(1:nagent).DirY]  = dummy{:};