% =========================
% EscapePanic      
% =========================
clear;

[Parameter,BuildingList,ExitList,Foldername,Topo_name] = SetupModel;


%plotting parameters
PLOTTING.Marking    = 'number'; %'number', 'smiley'
PLOTTING.FontSize   = 11;
PLOTTING.Color      = 'y';      %agents color


% workflow control
PlotSetup = false;
PlotEvolution = true;
save_time = 5; % after how many timesteps is an output file saved?


% Workflow control
DirectExitPath = Parameter.DirectExitPath;
WithAgents = Parameter.WithAgents;
WithTopo   = Parameter.WithTopo;
WithFlood  = Parameter.WithFlood;


% subfolder and topo information
Foldername = 'test'; % subfolder where output is to be stored



%==========================================================================
% add necessary paths
%==========================================================================       
addpath ./DecisionStrategy/
addpath ./WallForces/
addpath ./Plotting/
addpath ./kdtree_alg_OSX/
addpath ./FastMarching_version3b, add_function_paths();

%==========================================================================
% initialize grid
%==========================================================================
resolution          = Parameter.resolution;
xmin                = Parameter.xmin;
xmax                = Parameter.xmax;
ymin                = Parameter.ymin;
ymax                = Parameter.ymax;

xvec                = xmin:resolution:xmax;
yvec                = ymin:resolution:ymax;
[X_Grid,Y_Grid]     = meshgrid(xvec,yvec);

% set topography
if strcmp(Topo_name,'none')
Z_Grid = -0*(sin(0.5*X_Grid)+cos(0.7*Y_Grid));
else
   load(Topo_name);
   Z_Grid = interp2(XTopo,YTopo,ZTopo,X_Grid,Y_Grid);
end
% compute topography gradient
[Gradient_x,Gradient_y] = gradient(Z_Grid,resolution,resolution);

%convert time
maxtime         = Parameter.maxtime*60; %[min] => [s]
decision_step   = round(Parameter.decision_time/Parameter.dt);

%---------------------------------------
% create starting area map for agents
%---------------------------------------
StartArea = zeros(size(yvec,2),size(xvec,2));
StartArea(X_Grid<5) = 1;

%---------------------------------------
% create boundary map for later use
%---------------------------------------
BoundaryMap = zeros(size(yvec,2),size(xvec,2));
BoundaryMap(1,:)=1; BoundaryMap(size(yvec,2),:)=1; BoundaryMap(:,1)=1; BoundaryMap(:,size(xvec,2))=1;

%---------------------------------------
% create building map for later use
%---------------------------------------
BuildingList(find(BuildingList(:,1)>=xmax),:) = []; %if building fully outside domain: remove it!
BuildingList(find(BuildingList(:,3)>=ymax),:) = []; %if building fully outside domain: remove it!
BuildingList(find(BuildingList(:,2)>xmax),2) = xmax; %adjust building to domain boundary
BuildingList(find(BuildingList(:,4)>ymax),2) = ymax; %adjust building to domain boundary

BuildingMap = logical(X_Grid*0);
% add buildings to map
for i=1:size(BuildingList,1)
    BuildingMap(X_Grid>=BuildingList(i,1) & X_Grid<=BuildingList(i,2) & Y_Grid>=BuildingList(i,3) & Y_Grid<=BuildingList(i,4)) = true;
end

%-----------------------------------------------------------------
% create exit map for later use and compute center point of exits
%-----------------------------------------------------------------
ExitList(find(ExitList(:,1)>=xmax),:)   = []; %if exit fully outside domain: remove it!
ExitList(find(ExitList(:,3)>=ymax),:)   = []; %if exit fully outside domain: remove it!
ExitList(find(ExitList(:,2)>xmax),2)    = xmax; %adjust exit to domain boundary
ExitList(find(ExitList(:,4)>ymax),2)    = ymax; %adjust exit to domain boundary

ExitMap = logical(X_Grid*0);
for i=1:size(ExitList,1)
    ExitMap(X_Grid>=ExitList(i,1) & X_Grid<=ExitList(i,2) & Y_Grid>=ExitList(i,3) & Y_Grid<=ExitList(i,4)) = true;
end


% initialize agents
nagent  = Parameter.nagent;
AGENT   = InitializeAgents(nagent,Parameter);

% create random agent distribution
AGENT   = CreateInitialAgentDistribution(nagent,AGENT,X_Grid,Y_Grid,BuildingMap,BoundaryMap,StartArea,ExitMap);

%----------------------------------------------------
% building locations
%----------------------------------------------------
x_Buildings = X_Grid(BuildingMap);
y_Buildings = Y_Grid(BuildingMap);


%----------------------------------------------------
% compute forces from buildings (static)
%----------------------------------------------------
[ArchForce,ArchDirX,ArchDirY] = ArchitectureForceV2(X_Grid,Y_Grid,BuildingList,Parameter,resolution);

%----------------------------------------------------
% compute shortest path to exit
%----------------------------------------------------
if (~DirectExitPath && ~WithTopo)
    % compute shortest path without topography with fast marchng algorithm
    [Dgradx,Dgrady,D_orig] = ComputeShortestPathGlobal(BuildingMap,ExitMap,X_Grid,Y_Grid,Parameter.v0,Parameter.resolution);
elseif (~DirectExitPath && WithTopo)
    % compute shortest path with topography with fast marchng algorithm
    [Dgradx,Dgrady,D_orig] = ComputeShortestPathGlobalTopo(BuildingMap,ExitMap,X_Grid,Y_Grid,Z_Grid,D_orig,Gradient_x,Gradient_y,Parameter);
elseif DirectExitPath
    % compute exit direction directly
    [Dgradx,Dgrady] = ComputeShortestPathGlobalDirect(ExitMap,X_Grid,Parameter.v0,Parameter.resolution);
end

%----------------------------------------------------
% plot setup
%----------------------------------------------------
if PlotSetup
    % plot setup
    figure(1),clf
    hold on
    % plot buildings
    PlotBuildings(BuildingList,'r');
    PlotBuildings(ExitList,'g');
    % plot agents
    %         PlotAgents(nagent,AGENT,'y');
    PlotAgents2(nagent,AGENT,PLOTTING);
    axis equal
    axis([min(X_Grid(:)) max(X_Grid(:)) min(Y_Grid(:)) max(Y_Grid(:))])
    box on
    title('time = 0.00 min')
    xlabel('x [m]')
    ylabel('y [m]')
end


%==========================================================================
% time loop

time=0; itime=0;
while (time <= maxtime && size(AGENT,2)>0)
    time = time+Parameter.dt;     %actual time
    itime = itime+1;    %nr. timesteps
    disp('*****************************************')
    disp(['timestep ',num2str(itime),':    time = ',num2str(time/60),' min'])
    
    if (nagent~=size(AGENT,2)); error('fc: nagent not equal nr. of agents!'); end

    %----------------------------------------------------
    % compute flooding
    %----------------------------------------------------
    if WithFlood
       error('not yet implemented') 
    end
    
    %----------------------------------------------------
    % compute kdtree of agents for later use
    %----------------------------------------------------
    ReferencePoints         = zeros(nagent,2);
    ReferencePoints(:,1)    = [AGENT(:).LocX];
    ReferencePoints(:,2)    = [AGENT(:).LocY];
    
    % generate tree
    tree =   kdtree(ReferencePoints);
    
    % clearvars ReferencePoints
    
    %----------------------------------------------------
    % compute forces from buildings on all agents (just interpolate the
    % precomputed force field to the agents)
    %----------------------------------------------------
    if Parameter.SocialForces
        [FxSocialWalls,FySocialWalls] = ComputeSocialForcesStatic(AGENT,X_Grid,Y_Grid,ArchForce,ArchDirX,ArchDirY,Parameter);
        dummy                                 = num2cell(FxSocialWalls);
        [AGENT(1:nagent).FxSocialWalls]       = dummy{:};
        dummy                                 = num2cell(FySocialWalls);
        [AGENT(1:nagent).FySocialWalls]       = dummy{:};
    else
        [AGENT(1:nagent).FxSocialWalls]       = deal(0);
        [AGENT(1:nagent).FySocialWalls]       = deal(0);
    end
    
    
    %----------------------------------------------------
    % compute direction field to exits on all agents 
    % (just interpolate the precomputed field to the agents)
    %----------------------------------------------------

    if (~DirectExitPath && WithAgents)
        if (mod(itime,decision_step)==0 || itime==1)
            [Dgradx,Dgrady] = ComputeShortestPathGlobalWithAgents(BuildingMap,ExitMap,X_Grid,Y_Grid,Z_Grid,D_orig,AGENT,nagent,Parameter);
        end
    elseif (~DirectExitPath && WithAgents && WithFlood)
        error('not yet implemented')
    end
    
    xExitDirAgents = interp2(X_Grid,Y_Grid,Dgradx,[AGENT.LocX],[AGENT.LocY],'*linear');
    yExitDirAgents = interp2(X_Grid,Y_Grid,Dgrady,[AGENT.LocX],[AGENT.LocY],'*linear');
    
    % normalize direction vector
    dirtot         = sqrt(xExitDirAgents.^2+yExitDirAgents.^2);
    xExitDirAgents = xExitDirAgents./dirtot;
    yExitDirAgents = yExitDirAgents./dirtot;
    
    dummy = num2cell(xExitDirAgents);
    [AGENT(1:nagent).xExitDir]       = dummy{:};
    dummy = num2cell(yExitDirAgents);
    [AGENT(1:nagent).yExitDir]       = dummy{:};
    

    %----------------------------------------------------
    % agent loop
    iagent=0; nagent2 = nagent;
    for iagent = 1:nagent2
        x_agent         = AGENT(iagent).LocX;
        y_agent         = AGENT(iagent).LocY;
        agent_size      = AGENT(iagent).Size;
        velx_agent      = AGENT(iagent).VelX;
        vely_agent      = AGENT(iagent).VelY;
        %-------------------------------------------------
        % check if the agent is outside the domain
        %-------------------------------------------------
        if x_agent>xmax || x_agent<xmin || y_agent>ymax || y_agent<ymin
           error('fc: stupid agent outside domain!') 
        end
        
        %-------------------------------------------------
        % get the agents that are in the "individual box" and compute the
        % distance to them
        %-------------------------------------------------
        [AGENT,x_others,y_others,others_size] = GetSurroundingAgents(iagent,AGENT,tree);
        
        [Normal,Tangent,DistanceToAgents,num_others] = ComputeDistanceToAgents(x_agent,y_agent,agent_size,x_others,y_others,others_size);
        if ~isempty(find(DistanceToAgents==0)); error('fc: dividing by zero!'); end

        if num_others >0
            % find agents that are too close
            indTooClose     = find(DistanceToAgents>=0);
            
            %----------------------------------------------------
            % compute social forces from other agents and apply a weighting
            % function to simulate that agents only have a reduced field of
            % vision
            %----------------------------------------------------
            if Parameter.SocialForces
                [FxAgentsSocial,FyAgentsSocial] = ComputeSocialForcesDynamic(Parameter,DistanceToAgents,Normal);
            else
                FxAgentsSocial = 0;
                FyAgentsSocial = 0;
            end
            %----------------------------------------------------
            % compute physical forces from other agents
            %----------------------------------------------------
            if (~isempty(indTooClose) && Parameter.PhysicalForces)
                surr_agents     = AGENT(iagent).SurroundingAgents;
                AgentTooClose   = surr_agents(indTooClose);
                DistTooClose    = DistanceToAgents(indTooClose);
                
                velx_others = AGENT(AgentTooClose).VelX;
                vely_others = AGENT(AgentTooClose).VelY;
                
                [FxPhysAgents,FyPhysAgents] = ComputePhysicalForceAgents(velx_agent,vely_agent,velx_others,vely_others,Parameter,DistTooClose,Normal(indTooClose,:),Tangent(indTooClose,:));
                
            else
                FxPhysAgents = 0;
                FyPhysAgents = 0;
            end
        else
            FxAgentsSocial = 0;
            FyAgentsSocial = 0;
            FxPhysAgents = 0;
            FyPhysAgents = 0;
        end
        %----------------------------------------------------
        % compute physical forces from walls 
        %----------------------------------------------------
        if Parameter.PhysicalForces
            [FxPhysWall,FyPhysWall] = ComputePhysicalForceWalls(x_agent,y_agent,agent_size,velx_agent,vely_agent,x_Buildings,y_Buildings,Parameter);
        else
            FxPhysWall = 0;
            FyPhysWall = 0;
        end
        %----------------------------------------------------
        % assign forces to structure
        %----------------------------------------------------
        AGENT(iagent).FxPhysAgents   = sum(FxPhysAgents);
        AGENT(iagent).FyPhysAgents   = sum(FyPhysAgents);
        AGENT(iagent).FxPhysWall     = sum(FxPhysWall);
        AGENT(iagent).FyPhysWall     = sum(FyPhysWall);
        
        % add some random noise on the social force from other agents
        AGENT(iagent).FxSocialAgents = sum(FxAgentsSocial)*(1+ Parameter.pert_social*(-0.5+rand(1)) );
        AGENT(iagent).FySocialAgents = sum(FyAgentsSocial)*(1+ Parameter.pert_social*(-0.5+rand(1)) );
    end
    
    %----------------------------------------------------
    % compute exit force
    %----------------------------------------------------
    [AGENT] = ComputeExitForce(AGENT,Parameter,nagent);
   
    %----------------------------------------------------
    % move agents
    %----------------------------------------------------
    [AGENT] = MoveAgents(AGENT,X_Grid,Y_Grid,Gradient_x,Gradient_y,Parameter.dt,nagent,Parameter);
    
    %----------------------------------------------------
    % remove successfull agents
    %----------------------------------------------------

    
    %those who arrived in the exits
    for i=1:size(ExitList,1)
        AGENT(   [AGENT.LocX]>=ExitList(i,1) & [AGENT.LocX]<=ExitList(i,2) ...
            & [AGENT.LocY]>=ExitList(i,3) & [AGENT.LocY]<=ExitList(i,4)  ) = [];
    end
    %remove agents outside model domain
    AGENT(   [AGENT.LocX]>xmax | [AGENT.LocX]<xmin ...
        | [AGENT.LocY]>=ymax | [AGENT.LocY]<ymin  ) = [];
    
    nagent = size(AGENT,2); %update number of agents after removing some of them
    cell_array = num2cell(1:nagent); [AGENT(1:nagent).num] = cell_array{:}; %update correct numbering from 1:nagent

    %----------------------------------------------------
    % save data
    %----------------------------------------------------
    if mod(itime,Parameter.SaveTimeStep)==0
        filestem = ['+output/',Foldername];
        if ~exist(filestem); mkdir(filestem); end
        
        filename_full = [filestem,'/',Foldername,'_',num2str(itime,'%5.6d')];
        
        save(filename_full,'AGENT')
    end
    
    %----------------------------------------------------
    % plot
    %----------------------------------------------------


    if (PlotEvolution && mod(itime,Parameter.PlotTimeStep)==0)

        figure(1),clf
        hold on
        pcolor(X_Grid,Y_Grid,Z_Grid),shading flat,colorbar
        % plot buildings
        PlotBuildings(BuildingList,'r');
        PlotBuildings(ExitList,'g');
        % plot agents
        
%         PlotAgents(nagent,AGENT,'y');
        PlotAgents2(nagent,AGENT,PLOTTING);
        
%        quiver([AGENT(1:nagent).LocX],[AGENT(1:nagent).LocY],[AGENT(1:nagent).xExitDir],[AGENT(1:nagent).yExitDir],'r')
%         quiver(X_Grid,Y_Grid,Dgradx,Dgrady,'w')
        axis equal
        axis([min(X_Grid(:)) max(X_Grid(:)) min(Y_Grid(:)) max(Y_Grid(:))])
        box on
        title(['time = ',num2str(time/60,3),' min'])
        xlabel('x [m]')
        ylabel('y [m]')
        
    end
end

%==========================================================================