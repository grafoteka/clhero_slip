function [fig] = plotting (sim_data,vel_objetivo,leg_size, walking_angle, running_angle,Kp,Ki)

%DATA LENGTH
data_length = length(sim_data);
vector_vel = repmat( vel_objetivo, [1,10000] );
x_vel_media = 0:0.1:8;
x0 = 0;
y0 = 0;

axis_velocityx=[0 10 0 1.3];
axis_torque=[0 10 -20 20];
axis_height=[0 10 0 0.4];
axis_velocityy=[0 10 -1.5 1.5];
axis_error=[0 10 -2 2];

fig = figure('visible','on');
%set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 1, 0.96]);
set(fig,'units','normalized','outerposition',[0 0 1 1]);
ax1 = subplot(5,1,1);
plot(ax1,x0,y0);
axis(axis_velocityx);
title('Velocity X');
ylabel('Forward velocity (m/s)','fontweight','bold');
grid on;
ax2 = subplot(5,1,2);
plot(ax2,x0,y0);
axis(axis_torque);
title('Torque');
ylabel('Hip torque (Nm)','fontweight','bold');
grid on;
ax3 = subplot(5,1,3);
plot(ax3,x0,y0);
axis(axis_height);
title('CoM height');
ylabel('CoM height (m)','fontweight','bold');
grid on;
ax4 = subplot(5,1,4);
plot(ax4,x0,y0);
axis(axis_velocityy);
title('Velocity Y');
ylabel('Vertical velocity (m/s)','fontweight','bold');
grid on;
ax5=subplot(5,1,5);
plot(ax5, x0,y0);
axis(axis_error);
title('Accumulated error');
xlabel('CoM forward displacement (m)','fontweight','bold');
ylabel('Accumulated error','fontweight','bold');
grid on;


for i=1:data_length

    hold(ax1,'on'); 
    if i == 1
      h =  plot(ax1,sim_data(i).q(:,1), sim_data(i).q(:,3),'color',[1 0.1 0.1],'linewidth', 1.5);

    else
        plot(ax1,sim_data(i).q(:,1), sim_data(i).q(:,3),'color',[1 0.1 0.1],'linewidth', 1.5);
        
    end
    hold(ax1,'off');
    hold(ax2,'on');
    plot(ax2,sim_data(i).q(:,1), sim_data(i).T,'color',[0.3 0.7 0.1], 'linewidth', 1.5);
    hold(ax2,'off');
    hold(ax3,'on');
    plot(ax3,sim_data(i).q(:,1), sim_data(i).q(:,2),'color',[0 0.4 1],'linewidth', 1.5);
    hold(ax3,'off');
    hold(ax4,'on');
    plot(ax4,sim_data(i).q(:,1), sim_data(i).q(:,4),'color',[1 0.5 0],'linewidth', 1.5);
    hold(ax4,'off');
    hold(ax5, 'on'),
%     plot(ax5, sim_data(i).q(:,1), sim_data(i).error_acum, 'color', [0 0 0], 'linewidth', 1.5);
    hold(ax5, 'off');
end
hold(ax1, 'on');
j =  plot(ax1,x_vel_media,vector_vel(1:length(x_vel_media)),'--', 'color',[0.5 0.5 0.5],'linewidth',1);
legend([h,j],{'real-time velocity','desired velocity'}, 'Location', 'Northeast');
hold(ax1,'off');
if leg_size == 0.203
    leg_type = 'Carbon fiber';
else
    leg_type = 'Glass fiber';
end
sgtitle(['Leg: ' leg_type '; Stance attack angle = ' num2str(walking_angle) 'º; Flight attack angle = ' num2str(running_angle) 'º; Desired forward velocity = ' num2str(vel_objetivo) ' m/s; Kp = ' num2str(Kp) ' ; Ki = ' num2str(Ki)],'fontweight','bold');  
end