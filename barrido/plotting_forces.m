%%=====================================================================

%Authors: Jorge de León Rivas and Rodrigo Sánchez Molina

%File: 'plotting_forces.m'

%Version: Beta 1.0

%Description: Function to plot CoM trajectories of the robot

%Changelog:

%%=====================================================================

function [fig] = plotting_forces (data, desired_vel,leg_size, walking_angle, running_angle,Kp)

    %calculating the data length to display every step of the data struct
    data_length = length(data);

    %reference
    x0 = 0;
    y0 = 0;

    %defining the axis_range of the plots
    axis_forcesx=[0 5 -100 100]; % [x_initial x_final y_initial y_final]
    axis_forcesy=[0 5 -100 100];

    %creating new figure
    fig = figure('visible','on');
    set(fig,'units','normalized','outerposition',[0 0 1 1]); % to make the figure size bigger

    %creating subplots of the figure
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

    % colors
    red = [1 0.1 0.1];
    green = [0.3 0.7 0.1];
    
    for i=1:data_length

        if strcmp(data(i).type,'double') % different color for double phase
            
            %plotting ax1
            hold(ax1,'on');
            plot(ax1,data(i).q(:,1), data(i).Fx,'color',red,'linewidth', 1.5);
            hold(ax1,'off');
            
            %plotting ax2
            hold(ax2,'on');
            plot(ax2,data(i).q(:,1), data(i).Fy,'color',red, 'linewidth', 1.5);
            hold(ax2,'off');

        else
            
            %plotting ax1
            hold(ax1,'on');
            plot(ax1,data(i).q(:,1), data(i).Fx,'color',green,'linewidth', 1.5);
            hold(ax1,'off');
            
            %plotting ax2
            hold(ax2,'on');
            plot(ax2,data(i).q(:,1), data(i).Fy,'color',green, 'linewidth', 1.5);
            hold(ax2,'off');

        end

    end

    %relevant info to display in the plot title
    if leg_size == 0.203

        leg_type = 'Carbon fiber';

    else

        leg_type = 'Glass fiber';

    end

    sgtitle(['Leg: ' leg_type '; Stance attack angle = ' num2str(walking_angle) 'º; Flight attack angle = ' num2str(running_angle) 'º; Desired forward velocity = ' num2str(desired_vel) ' m/s; Kp = ' num2str(Kp) ],'fontweight','bold');  

end