%%=====================================================================

%Authors: Jorge de León Rivas and Rodrigo Sánchez Molina

%File: 'plotting.m'

%Version: Beta 1.0

%Description: Function to plot CoM trajectories of the robot

%Changelog:

%%=====================================================================

function [fig] = plotting (data,desired_vel,leg_size, walking_angle, running_angle,Kp)

    %calculate de data lenght to display every step of the data struct
    data_length = length(data);
    
    %relevant data to plot the desired forward velocity
    velocity_vector = repmat( desired_vel, [1,10000] );
    x_mean_velocity = 0:0.1:8;
    
    %reference
    x0 = 0;
    y0 = 0;

    %defining the axis_range of the plots
    axis_velocityx = [0 5 0 0.8]; % [x_initial x_final y_initial y_final]
    axis_torque = [0 5 -10 10];
    axis_height = [0 5 0 0.25];
    axis_velocityy = [0 5 -1.5 1.5];

    %creating new figure
    fig = figure('visible','on');
    set(fig,'units','normalized','outerposition',[0 0 1 1]); % to make the figure size bigger
    
    %creating subplots of the figure
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
    ylabel('Vertical velocity (m/s)','fontweight','bold');
    grid on;

    %colors
    red = [1 0.1 0.1];
    green = [0.3 0.7 0.1];
    blue = [0 0.4 1];
    orange = [1 0.5 0];
    grey = [0.5 0.5 0.5];
    %loop to plot sim_data steps
    for i=1:data_length
        
        %plotting ax1
        hold(ax1,'on'); 
        if i == 1
          h =  plot(ax1,data(i).q(:,1), data(i).q(:,3),'color',red,'linewidth', 1.5);

        else
            plot(ax1,data(i).q(:,1), data(i).q(:,3),'color',red,'linewidth', 1.5);

        end
        hold(ax1,'off');
        
        %plotting ax2
        hold(ax2,'on');
        plot(ax2,data(i).q(:,1), data(i).T,'color',green, 'linewidth', 1.5);
        hold(ax2,'off');
        
        %plotting ax3
        hold(ax3,'on');
        plot(ax3,data(i).q(:,1), data(i).q(:,2),'color',blue,'linewidth', 1.5);
        hold(ax3,'off');
        
        %plotting ax4
        hold(ax4,'on');
        plot(ax4,data(i).q(:,1), data(i).q(:,4),'color',orange,'linewidth', 1.5);
        hold(ax4,'off');

    end
    
    %creating ax1 label
    hold(ax1, 'on');
    j =  plot(ax1,x_mean_velocity,velocity_vector(1:length(x_mean_velocity)),'--', 'color',grey,'linewidth',1);
    legend([h,j],{'real-time velocity','desired velocity'}, 'Location', 'Northeast');
    hold(ax1,'off');
    
    %relevant info to display in the plot title
    if leg_size == 0.203
        leg_type = 'Carbon fiber';
    else
        leg_type = 'Glass fiber';
    end
    
    sgtitle(['Leg: ' leg_type '; Stance attack angle = ' num2str(walking_angle) 'º; Flight attack angle = ' num2str(running_angle) 'º; Desired forward velocity = ' num2str(desired_vel) ' m/s; Kp = ' num2str(Kp) ],'fontweight','bold');  

end