function [fig] = plotting_forces (sim_data, vel_objetivo,leg_size, walking_angle, running_angle,Kp)

%DATA LENGTH
data_length = length(sim_data);

x0 = 0;
y0 = 0;

axis_forcesx=[0 5 -100 100];
axis_forcesy=[0 5 -100 100];


fig = figure('visible','on');

set(fig,'units','normalized','outerposition',[0 0 1 1]);
ax1 = subplot(2,1,1);
plot(ax1,x0,y0);
axis(axis_forcesx);
title('X-Resultant  in the CoM');
ylabel('Force (N)','fontweight','bold');
grid on;
ax2 = subplot(2,1,2);
plot(ax2,x0,y0);
axis(axis_forcesy);
title('Y-Resultant force in the CoM');
ylabel('Force (N)','fontweight','bold');
xlabel('CoM forward displacement');
grid on;


for i=1:data_length
     
    if strcmp(sim_data(i).type,'double')
        hold(ax1,'on');
        plot(ax1,sim_data(i).q(:,1), sim_data(i).Fx,'color',[1 0.1 0.1],'linewidth', 1.5);
        hold(ax1,'off');
        hold(ax2,'on');
        plot(ax2,sim_data(i).q(:,1), sim_data(i).Fy,'color',[1 0.1 0.1], 'linewidth', 1.5);
        hold(ax2,'off');
        
    else
        
        hold(ax1,'on');
        plot(ax1,sim_data(i).q(:,1), sim_data(i).Fx,'color',[0.3 0.7 0.1],'linewidth', 1.5);
        hold(ax1,'off');
        hold(ax2,'on');
        plot(ax2,sim_data(i).q(:,1), sim_data(i).Fy,'color',[0.3 0.7 0.1], 'linewidth', 1.5);
        hold(ax2,'off');
        
    end
    
end

if leg_size == 0.203
    
    leg_type = 'Carbon fiber';
    
else
    
    leg_type = 'Glass fiber';
    
end

sgtitle(['Leg: ' leg_type '; Stance attack angle = ' num2str(walking_angle) 'º; Flight attack angle = ' num2str(running_angle) 'º; Desired forward velocity = ' num2str(vel_objetivo) ' m/s; Kp = ' num2str(Kp) ],'fontweight','bold');  

end