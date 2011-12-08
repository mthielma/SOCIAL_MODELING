
%===========================
% 
% plotting output files
% 
%=========================== 
clear;

%-- input -----------------------------------------

filename            = 'Model2';

filestem            = ['../+output/',filename,'/'];

savingPlots = logical(1);   save_jpg = logical(0);    save_eps = logical(1);

FontSize            = 14;
AgentsMarking       = 'none';           % 'none', 'number', 'smiley'
AgentsColor         = 'rand';           % agents color: 'y',[0 1 0], or 'rand'

ColorBuildings      = [0.2 0.2 0.2];
ColorExits          = [0.0 0.4 0.0];

%--------------------------------------------------


display('***************')
display(['saving output files for ',filename])

filestem_full = [filestem,'Setup.mat'];
if exist(filestem_full,'file')
    load(filestem_full)
else
    error(['Could not find ',filestem_full,' !']);
end


% agent settings
nagent = Parameter.nagent; %initial number of agents

% plotting settings
Plotting.FontSize	= FontSize;
Plotting.Marking  	= AgentsMarking;
Plotting.Color    	= AgentsColor;

cmap = hsv(nagent);  %# Creates a nagent-by-3 set of colors from the HSV colormap
if strcmp(Plotting.Color,'rand')
    Plotting.cmap   = cmap;
end

% time settings
maxTime     = Parameter.maxtime*60;    %[s]
dt          = Parameter.dt;

outputStep  = Parameter.SaveTimeStep;
nrTimesteps = maxTime/dt;               %max. number of timesteps (if it did run until maxTime)
nrFiles     = nrTimesteps/outputStep;   %max. number of output files (if it did run until maxTime)
% dtFiles     = dt * outputStep;        %timestep between output files [s]

%loop output files
for i=0:outputStep:nrTimesteps
    time = i*dt; %time in [s]
    
    num_string = num2str(100000+i);
    num_string(1)='0';
    filestem_full = [filestem,filename,'_',num_string,'.mat'];
    
    
    if exist(filestem_full,'file')
        load(filestem_full)
        
        
        figure(1),clf
        set(cla,'FontSize',Plotting.FontSize)
        %pcolor(X_Grid,Y_Grid,Z_Grid),shading flat,colorbar
        % plot buildings
        PlotBuildings(BuildingList,ColorBuildings);
        PlotBuildings(ExitList,ColorExits);
        % plot agents
        PlotAgents(AGENT,Plotting);
        
        % quiver([AGENT(1:nagent).LocX],[AGENT(1:nagent).LocY],[AGENT(1:nagent).xExitDir],[AGENT(1:nagent).yExitDir],'r')
        % quiver(X_Grid,Y_Grid,Dgradx,Dgrady,'b')
        % quiver([AGENT.LocX],[AGENT.LocY],[AGENT.DirX],[AGENT.DirY],'r-')
        axis equal
%         axis([min(X_Grid(:)) max(X_Grid(:)) min(Y_Grid(:)) max(Y_Grid(:))])
        box on
%         title(['time = ',num2str(time,'%.2d'),' s'])
        xlabel('x [m]')
        ylabel('y [m]')
        
        if time/60<1; title(['time = ',num2str(time,3),' s']);
        else title(['time = ',num2str(time/60,3),' min']); end
        
        %saving plots
        if savingPlots
            filestem_save = ['../+output/',filename,'/+images'];
            if ~exist(filestem_save,'dir'); mkdir(filestem_save); end
            
            filenameIM = [filestem_save,'/',filename,'_',num_string];
            if save_jpg; print(filenameIM,'-djpeg90','-r300'); end
            if save_eps; print(filenameIM,'-depsc'); end
        end
        
    end
    
    
end

display('finished plotting output files.')
display('***************')