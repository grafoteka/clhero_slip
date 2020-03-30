l = 1;
vel_objetivo = 0.5;
stiffness = 5000;
leg_size = 0.203;
compresion_inicial = leg_size - 10*9.8/stiffness;
running_angle = 100;
walking_angle = 64;
time = 15;


vector_vel = repmat( vel_objetivo, [1,10000] );
x_vel_media = 0:0.1:8;
x0 = 0;
y0 = 0;

axis_velocityx=[0 2 0 0.8];
axis_torque=[0 2 -3.5 3.5];
axis_height=[0 2 0 0.25];
axis_velocityy=[0 2 -1 1];

fig = figure('visible','on');
set(fig,'units','normalized','outerposition',[0 0 1 1]);
ax1 = subplot(4,1,1);
plot(ax1,x0,y0);
axis(axis_velocityx);
title('Velocity X');
ylabel('Forward velocity (m/s)','fontweight','bold');
grid on;
ax2 = subplot(4,1,2);
plot(ax2,x0,y0);
axis(axis_torque);
title('Torque');
ylabel('Hip torque (Nm)','fontweight','bold');
grid on;
ax3 = subplot(4,1,3);
plot(ax3,x0,y0);
axis(axis_height);
title('CoM height');
ylabel('CoM height (m)','fontweight','bold');
grid on;
ax4 = subplot(4,1,4);
plot(ax4,x0,y0);
axis(axis_velocityy);
title('Velocity Y');
xlabel('CoM forward displacement (m)','fontweight','bold');
ylabel('Vertical velocity (m/s)','fontweight','bold');
grid on;

p = [];
q = [];
r = [];
s = [];
rojo = [1 0.1 0.1];
verde = [0.3 0.7 0.1];
azul = [0 0.4 1];

while (l<4)
    if l == 1
        Kp(l) = 1;
        color = rojo;
    end
    if l == 2
        Kp(l) = 100;
        color = azul;
    end
    if l == 3
        Kp(l) = 1000;
        color = verde;
    end

    
    sim_data=SLIP_model([0,compresion_inicial, vel_objetivo, 0, running_angle, walking_angle, stiffness], vel_objetivo, time, leg_size, Kp(l)); 
    disp(['Kp : ' num2str(Kp(l)) ' leg_size : ' num2str(leg_size)]);
    data_length = length(sim_data);

    for i=1:data_length
        
     
        if i == 1
            hold(ax1,'on');
            p(l)=plot(ax1,sim_data(i).q(:,1), sim_data(i).q(:,3),'color',color,'linewidth', 1);
            hold(ax1,'off');
            hold(ax2,'on');
            q(l)=plot(ax2,sim_data(i).q(:,1), sim_data(i).T,'color',color, 'linewidth', 1);
            hold(ax2,'off');
            hold(ax3,'on');
            r(l)=plot(ax3,sim_data(i).q(:,1), sim_data(i).q(:,2),'color',color,'linewidth', 1);
            hold(ax3,'off');
            hold(ax4,'on');
            s(l)=plot(ax4,sim_data(i).q(:,1), sim_data(i).q(:,4),'color',color,'linewidth', 1);
            hold(ax4,'off');
        else
            hold(ax1,'on');
            plot(ax1,sim_data(i).q(:,1), sim_data(i).q(:,3),'color',color,'linewidth', 1);
            hold(ax1,'off');
            hold(ax2,'on');
            plot(ax2,sim_data(i).q(:,1), sim_data(i).T,'color',color, 'linewidth', 1);
            hold(ax2,'off');
            hold(ax3,'on');
            plot(ax3,sim_data(i).q(:,1), sim_data(i).q(:,2),'color',color,'linewidth', 1);
            hold(ax3,'off');
            hold(ax4,'on');
            plot(ax4,sim_data(i).q(:,1), sim_data(i).q(:,4),'color',color,'linewidth', 1);
            hold(ax4,'off');     
        end
    

    end
            
    l = l+1;
end


hold(ax1, 'on');
j =  plot(ax1,x_vel_media,vector_vel(1:length(x_vel_media)),'--', 'color',[0.5 0.5 0.5],'linewidth',1);
legend([p(1), p(2), p(3),j],{['Kp=' num2str(Kp(1))],['Kp=' num2str(Kp(2))],['Kp=' num2str(Kp(3))],'desired velocity'}, 'Location', 'Northeast');
hold(ax1,'off');
hold(ax2,'on');
legend([q(1), q(2), q(3)],{['Kp=' num2str(Kp(1))],['Kp=' num2str(Kp(2))],['Kp=' num2str(Kp(3))]}, 'Location', 'Northeast');
hold(ax2,'off');
hold(ax3,'on');
legend([r(1), r(2), r(3)],{['Kp=' num2str(Kp(1))],['Kp=' num2str(Kp(2))],['Kp=' num2str(Kp(3))]}, 'Location', 'Northeast');
hold(ax3,'off');
hold(ax4,'on');
legend([s(1), s(2), s(3)],{['Kp=' num2str(Kp(1))],['Kp=' num2str(Kp(2))],['Kp=' num2str(Kp(3))]}, 'Location', 'Northeast');
hold(ax4,'off');
if leg_size == 0.203
    leg_type = 'Carbon fiber';
else
    leg_type = 'Glass fiber';
end
sgtitle(['Leg: ' leg_type '; Stance attack angle = ' num2str(walking_angle) 'º; Flight attack angle = ' num2str(running_angle) 'º; Desired forward velocity = ' num2str(vel_objetivo) ' m/s'],'fontweight','bold');  
% matfilename=['torque/',num2str(leg_size,'%.3f'),'/SLIP_torque_kd'];
% print(fig,matfilename,'-dpng','-r600');