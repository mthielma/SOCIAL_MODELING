
%===========================
% 
% plotting analysis files
% 
%=========================== 
clear;

%-- input -----------------------------------------

filename            = 'BeachEvacuationOneExitStreetWidth1_Flood0_1';

filestem            = ['../+output/',filename,'/'];

savingPlots = logical(1);   save_jpg = logical(1);    save_eps = logical(1);

FontSize            = 14;
AgentsMarking       = 'none';       	% 'none', 'number', 'smiley'
AgentsColor         = [0.75 0.75 0];           % agents color: 'y' or [0.75 0.75 0] or 'rand' or 'one'


%to follow one single agent:   set AgentsColor = 'one'
Don                 = 1;                %insert name of Don
ColorDon            = [1.0 0.0 0.0];
ColorOthers         = [0.5 0.5 0.5];
%--------------------------------------------------


display('***************')
if savingPlots; display(['saving analysis output files for ',filename]); end

filestem_full = [filestem,'Setup.mat'];
if exist(filestem_full,'file')
    load(filestem_full)
else
    error(['Could not find ',filestem_full,' !']);
end

filestem_full = [filestem,'/Analysis.mat'];
if exist(filestem_full,'file')
    load(filestem_full)
else
    error(['Could not find ',filestem_full,' !']);
end


%setup analysis variable
%Analysis = [num/name startPosX startPosY ExitTime Status]  
%                                       Status:	:   1:  'alive' still running
%                                                   2:  'survived' reached exit 
%                                                   3:  'killed' e.g. by flood




% agent settings
nagent = Parameter.nagent; %initial number of agents

% plotting settings
Plotting.FontSize	= FontSize;
Plotting.Marking  	= AgentsMarking;
Plotting.Color    	= AgentsColor;



% time settings
maxTime     = Parameter.maxtime*60;    %[s]
dt          = Parameter.dt;
 
outputStep  = Parameter.SaveTimeStep;
nrTimesteps = maxTime/dt;               %max. number of timesteps (if it did run until maxTime)
nrFiles     = nrTimesteps/outputStep;   %max. number of output files (if it did run until maxTime)
% dtFiles     = dt * outputStep;        %timestep between output files [s]



%find all agents still running (=alive)
alive    = Analysis(Analysis(:,5)==1 ,1);
Runners   = Analysis(alive);
%find all agents reaching the exit
survived    = Analysis(Analysis(:,5)==2 ,1);
Survivors   = Analysis(survived);
%find all agents captured by flood
drowned     = Analysis(Analysis(:,5)==3 ,1);
Drowners    = Analysis(drowned);

%cumulative no. agents per time
%alive
tstep = 1;
i=0; Num_old=size(Analysis,1);
for ii=0:tstep:max(Analysis(:,4))
    i = i+1;
    
    Num = size(Runners(Analysis(alive,4)>=ii & Analysis(alive,4)<ii+tstep) ,1);
    cumNumberVecA(i,1) = Num_old-Num;
    timeVector(i,1) = ii;
    
    Num_old = Num_old-Num;
end
%exits
tstep = 1;
i=0; Num_old=0;
for ii=0:tstep:max(Analysis(:,4))
    i = i+1;
    
    Num = size(Survivors(Analysis(survived,4)>=ii & Analysis(survived,4)<ii+tstep) ,1);
    cumNumberVecE(i,1) = Num_old+Num;
    timeVector(i,1) = ii;
    
    Num_old = Num_old+Num;
end
%drownings
i=0; Num_old=0;
for ii=0:tstep:max(Analysis(:,4))
    i = i+1;
    
    Num = size(Drowners(Analysis(drowned,4)>=ii & Analysis(drowned,4)<ii+tstep) ,1);
    cumNumberVecD(i,1) = Num_old+Num;
    timeVector(i,1) = ii;
    
    Num_old = Num_old+Num;
end
%total ~= runners
cumNumberVecT = cumNumberVecE+cumNumberVecD;

figure(1),clf
set(cla,'FontSize',Plotting.FontSize)


% no. exits vs. time
% halive=plot(timeVector,cumNumberVecA,'g');
% hold on
htotal=plot(timeVector,cumNumberVecT,'k');
hold on
hexit=plot(timeVector,cumNumberVecE,'b');
hold on
hflood=plot(timeVector,cumNumberVecD,'r');





box on
%         title(['time = ',num2str(time,'%.2d'),' s'])
xlabel('time [s]')
ylabel('cumulative number of agents')
grid on
legend([htotal hexit hflood],'total','exits','drowned','Location','NorthWest')




%saving plots
if savingPlots
    filestem_save       = ['../+output/',filename,'/+images'];
    filestem_save_eps   = ['../+output/',filename,'/+images/+eps'];
    if ~exist(filestem_save,'dir'); mkdir(filestem_save); end
    filenameIM = [filestem_save,'/Analysis_',filename];
    filenameIMeps = [filestem_save_eps,'/Analysis_',filename];
    
    if save_jpg; print(filenameIM,'-djpeg90','-r300'); end
    if save_eps; if ~exist(filestem_save_eps,'dir'); mkdir(filestem_save_eps); end
        print(filenameIMeps,'-depsc2','-painters');
    end
end




display('finished plotting analysis output files.')
display('***************')