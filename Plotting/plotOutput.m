
%===========================
% 
% plotting output files
% 
%=========================== 
clear;

%-- input -----------------------------------------

filename            = 'beach_1';

filestem            = ['../+output/',filename,'/'];

savingPlots = logical(1);   save_jpg = logical(1);    save_eps = logical(1);

Dimension           = 2;                % 2: 2-D   or   3: 3-D
FontSize            = 14;
AgentsMarking       = 'none';       	% 'none', 'number', 'smiley'
AgentsColor         = [0.75 0.75 0];           % agents color: 'y' or [0.75 0.75 0] or 'rand' or 'one'

ColorBuildings      = [0.2 0.2 0.2];
MarkingBuildings    = '';
buildingHeight      = 3.5;              %times agent height e.g. 1.5
ColorExits          = [0.0 0.4 0.0];
MarkingExits        = 'EXIT';

%to follow one single agent:   set AgentsColor = 'one'
Don                 = 1;                %insert name of Don
ColorDon            = [1.0 0.0 0.0];
ColorOthers         = [0.5 0.5 0.5];

%--------------------------------------------------


display('***************')
if savingPlots; display(['saving output files for ',filename]); end

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

if strcmp(Plotting.Color,'rand')
    cmap = hsv(nagent);  %# Creates a nagent-by-3 set of colors from the HSV colormap
elseif strcmp(Plotting.Color,'one')
    cmap = zeros(50,3);
    for k=1:nagent
        cmap(k,:) = ColorOthers;
    end
    cmap(Don,:) = ColorDon;
else
    cmap = 0; %just to define it
end
Plotting.cmap   = cmap;


% time settings
maxTime     = Parameter.maxtime*60;    %[s]
dt          = Parameter.dt;
 
outputStep  = Parameter.SaveTimeStep;
nrTimesteps = maxTime/dt;               %max. number of timesteps (if it did run until maxTime)
nrFiles     = nrTimesteps/outputStep;   %max. number of output files (if it did run until maxTime)
% dtFiles     = dt * outputStep;        %timestep between output files [s]



%loop output files
i_output = 0; pathDon = zeros(nrFiles,2)*NaN;
for i=0:outputStep:nrTimesteps
% for i=3000:3000
    time = i*dt; %time in [s]
    
    num_string = num2str(100000+i);
    num_string(1)='0';
    filestem_full = [filestem,'/',filename,'_',num_string,'.mat'];
    
    
    if exist(filestem_full,'file')
        load(filestem_full)
        i_output = i_output+1;

        
        figure(1),clf
        set(cla,'FontSize',Plotting.FontSize)
        
        if Dimension==2
            set(cla,'XGrid','on','YGrid','on');
            %pcolor(X_Grid,Y_Grid,Z_Grid),shading flat,colorbar
            % plot buildings
            if sum(sum(Z_Grid))~=0
%                 h = contourf(X_Grid,Y_Grid,Z_Grid,40,'EdgeColor','none');
%                 colorbar
%                 colormap('gray')

                PlotFlood(Parameter,X_Grid,Y_Grid,Z_Grid,time)
                hold on
                [c,h] = contour(X_Grid,Y_Grid,Z_Grid,'LineColor',[0.4 0.4 0.4],'LineStyle','--');
                hold on
                
            end
            PlotBuildings(BuildingList,ColorBuildings,'');
            PlotBuildings(ExitList,ColorExits,MarkingExits);
            
            if strcmp(AgentsColor,'one') %follow one's path
                if ~isempty(find([AGENT.name]==Don)) %Don's still alive
                    pathDon(i_output,:) = [ [AGENT([AGENT.name]==Don).LocX] [AGENT([AGENT.name]==Don).LocY] ];
                end
                hold on; plot(pathDon(:,1),pathDon(:,2),'Color',ColorDon)
            end
            
            % plot agents
            PlotAgents(AGENT,Plotting);
            
            % quiver([AGENT(1:nagent).LocX],[AGENT(1:nagent).LocY],[AGENT(1:nagent).xExitDir],[AGENT(1:nagent).yExitDir],'r')
            % quiver(X_Grid,Y_Grid,Dgradx,Dgrady,'b')
            % quiver([AGENT.LocX],[AGENT.LocY],[AGENT.DirX],[AGENT.DirY],'r-')
            axis equal
            axis([Parameter.xmin Parameter.xmax Parameter.ymin Parameter.ymax])
            box on
            %         title(['time = ',num2str(time,'%.2d'),' s'])
            xlabel('x [m]')
            ylabel('y [m]')
            grid on
        
        elseif Dimension==3
            Parameter.buildingHeight  = Parameter.AgentSize*buildingHeight;
            set(cla,'XGrid','on','YGrid','on');
            %PlotBuildings3D_Topo(Parameter,BuildingList,ColorBuildings,Z_Grid,MarkingBuildings)
            %PlotBuildings3D(Parameter,BuildingList,ColorBuildings,MarkingBuildings)
            %PlotBuildings3D(Parameter,ExitList,ColorExits,MarkingExits); hold on;
            if sum(sum(Z_Grid))~=0
                PlotTopography3D(X_Grid,Y_Grid,Z_Grid)
            end
            PlotAgents3D(Parameter,Plotting,AGENT,Z_Grid)
            if sum(sum(Z_Grid))~=0
                PlotFlood3D(Parameter,X_Grid,Y_Grid,Z_Grid,time)
            end
            PlotBuildings3D(Parameter,BuildingList,ColorBuildings,MarkingBuildings)
            PlotBuildings3D(Parameter,ExitList,ColorExits,MarkingExits); hold on;
            % camlight left;
            
            if strcmp(AgentsColor,'one') %follow one's path
                if ~isempty(find([AGENT.name]==Don)) %Don's still alive
                    pathDon(i_output,:) = [ [AGENT([AGENT.name]==Don).LocX] [AGENT([AGENT.name]==Don).LocY] ];
                end
                ind = ~isnan(pathDon(:,1));
                pathZ = interp2(X_Grid,Y_Grid,Z_Grid,pathDon(ind,1),pathDon(ind,2));
                hold on; plot3(pathDon(ind,1),pathDon(ind,2),pathZ,'Color',ColorDon)  %only for z-level==0 !!!
                % set viewpoint
                view([-82 14])
            end
            axis equal
            xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
        else
            error('fc: Unknown Dimension!')
        end
        
        
        
        
            
        if time/60<1; title(['time = ',num2str(time,3),' s']);
        else title(['time = ',num2str(time/60,3),' min']); end
        
        %saving plots
        if savingPlots
            filestem_save       = ['../+output/',filename,'/+images'];
            filestem_save_eps   = ['../+output/',filename,'/+images/+eps'];
            if ~exist(filestem_save,'dir'); mkdir(filestem_save); end
            if Dimension==2; 
                filenameIM = [filestem_save,'/',filename,'_',num_string]; 
                filenameIMeps = [filestem_save_eps,'/',filename,'_',num_string];
            elseif Dimension==3; 
                filenameIM = [filestem_save,'/Dim3_',filename,'_',num_string]; 
                filenameIMeps = [filestem_save_eps,'/Dim3_',filename,'_',num_string]; 
            end
            
            if save_jpg; print(filenameIM,'-djpeg90','-r300'); end
            if save_eps; if ~exist(filestem_save_eps,'dir'); mkdir(filestem_save_eps); end
                print(filenameIMeps,'-depsc2','-painters'); 
            end
        end
        
    end
    
    
end

display('finished plotting output files.')
display('***************')