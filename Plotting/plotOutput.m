
%===========================
% 
% plotting output files
% 
%=========================== 
clear;

filename            = 'Model2';

filestem            = ['../+output/',filename,'/'];

Plotting.FontSize	= 14;
Plotting.Marking  	= 'none';           % 'none', 'number', 'smiley'
Plotting.Color    	= 'y';              % agents color






filestem_full = [filestem,'Setup.mat'];
if exist(filestem_full,'file')
    load(filestem_full)
else
    error(['Could not find ',filestem_full,' !']);
end

%loop output files
for i=1:1:99999
    
    num_string = num2str(100000+i);
    num_string(1)='0';
    filestem_full = [filestem,filename,'_',num_string,'.mat'];
    
    
    if exist(filestem_full,'file')
        load(filestem_full)
        
        
        figure(1),clf
        set(cla,'FontSize',Plotting.FontSize)
        %pcolor(X_Grid,Y_Grid,Z_Grid),shading flat,colorbar
        % plot buildings
        PlotBuildings(BuildingList,'r');
        PlotBuildings(ExitList,'g');
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
    end
    
    
end