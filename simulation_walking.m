%%=====================================================================

%Authors: Jorge de León Rivas and Rodrigo Sánchez Molina

%File: 'simulation_walking.m'

%Version: Beta 1.0

%Description: Simulation of walking gaits of the TD-SLIP template,
%which has implemented a P controller of the forward velocity. The script
%sweeps different inputs such as controller gain, desired forward velocity
%and the attack angle. Relevant info is plotted such as centre of mass
%trajectories, dynamics, and stable vs non stable solution acording to 
%the walking angle.

%Changelog:

%%=====================================================================

%% Clean workspace and undesired data.

clc;
clear;


%% Mechanical parameters of the robot

leg_size = 0.203; % m
stiffness = 5000; % N/m
mass = 10/3; % Kg


%% Initial conditions

initial_compression = mass*9.8/stiffness; % m
desired_vel = 0.5; % m/s
v_y = 0; % m/s
v_x = desired_vel; % m/s
x_initial = 0; % m
y_initial = leg_size - initial_compression; % m
angle_running = NaN; % attack angle from jumping in degrees
angle_walking = 45; % attack angle from walking in degrees
Kp = 50; % initial pre-defined proportional gain of the controller (N*s)

%% Simulation time

maxtime = 15; % seconds

%% Initialization of the counters of the loops

counter_success = 0; % counter of the succesful gaits during each gain controller swept
counter_velocities = 0; % counter of the iterations during each gain controller swept
iterations = 0; % counter of the iterations during each gain controller swept
counter_controller = 0; % counter of controller swept iterations
counter_successful_gains = 0; % counter of every successful gait during all the simulations
counter_error = 0; % counter of non successfull gaits

%% Sweeping input variables

  for Kp= 0:10:80 % controller swept
      
    counter_controller = counter_controller + 1;
    iterations = 0;
    counter_success = 0;
    counter_velocities=0;
    counter_error=0;
    
    disp_kp = ['Testing for Kp = ', num2str(Kp)];
    disp(disp_kp);
      
    for desired_vel=0.1:0.1:0.7 % desired forward velocity swept

        v_x = desired_vel;
    
        counter_velocities = counter_velocities + 1;
        
        disp_text = ['Testing for Kp = ', num2str(Kp), ' --- Desired fordward velocity = ', num2str(desired_vel)];
        disp(disp_text);

        

        for angle_walking=40:90 % angle walking swept
            
            iterations = iterations + 1;
            
            % calling the TD-SLIP function
            [data,performance, result]=SLIP_model_walking([x_initial,y_initial,v_x,v_y,angle_walking,stiffness,mass],desired_vel, maxtime,leg_size,Kp); 

            if(result > 0) % treating successful gaits
                
                counter_success = counter_success + 1;
                counter_successful_gains = counter_successful_gains + 1;
                
                test_result = ['Successful test ended with Kp = ', num2str(Kp), ' -- Desired forward velocity: ', num2str(desired_vel),' -- Angle walking = ', num2str(angle_walking), ' -- Success = ', num2str(counter_success), '/', num2str(iterations)];
                disp(test_result);
                
                % storing succesful data
                successful_vector_gains(counter_successful_gains,:) = Kp; % to make a histogram of the best controller proportional gain
                successful_angles(counter_success,counter_velocities) = angle_walking; % keeps succesful angles during each Kp iteration
                successful_velocities(counter_success,counter_velocities)= desired_vel; % keeps succesful angles during each Kp iteration

               

                 % plotting successful data. Uncomment to plot during each test.
                  plotting(data, desired_vel, leg_size, angle_walking, angle_running, Kp);  % plotting CoM trajectories
                  plotting_forces(data,desired_vel,leg_size, angle_walking, angle_running,Kp); % plotting forces that suffers the CoM
                  pause;  

            else 
                
                counter_error = counter_error + 1;
                
                if strcmp(performance.terminationmsg, 'Body flight')
                    
                    angles_flight(counter_error,counter_velocities) = angle_walking;
                    velocity_flight(counter_error,counter_velocities) = desired_vel;

                else
                    angles_n(counter_error,counter_velocities) = angle_walking;
                    velocity_n(counter_error,counter_velocities) = desired_vel;
                    
               end% if srtcmp
               
           end% if(result > 0)
            
        end %for angle_walking
        
        disp_text = ['Test ended for Kp = ',num2str(Kp), ' -- Desired forward velocity = ', num2str(desired_vel), ' -- Success = ', num2str(counter_success), '/', num2str(iterations)];

    end %for desired_vel

    test_result = ['Test ended for Kp = ', num2str(Kp), ' -- Success = ', num2str(counter_success), '/', num2str(iterations)];
    disp(test_result);

    %plotting successful vs non successful gaits according to the angle walking and velocity
    walking(successful_angles, successful_velocities, angles_n, velocity_n,angles_flight, velocity_flight, Kp, counter_success ,iterations);
%    pause;
    
  end %for controller Kp

kp_optim(successful_vector_gains); %plotting results for kp altogheter

 
 
%% FUNCTIONS




%% Function to plot walking angles against velocity. Successful are distinguished from unstable gaits

function  walking (walking, velocity, angles_n, velocity_n,angles_flight, velocity_flight, Kp, counter_success,iterations)
    
    figure
    hold on;
    
    for k = 1:length(walking(1,:))   
        scatter(walking(:,k),velocity(:,k),'g', 'filled');
    end 
    
    for k = 1:length(angles_n(1,:))
        scatter(angles_n(:,k), velocity_n(:,k),'r', 'filled');
    end
    
    for k = 1:length(angles_flight(1,:))
        scatter(angles_flight(:,k), velocity_flight(:,k),'b', 'filled');
    end
    
    grid on;
    axis([40 90 0 0.8]);
    xlabel('Walking attack angle')
    ylabel('Desired forward velocity')
    title_text = ['Results for Kp = ', num2str(Kp), ' -- Success: ', num2str(counter_success), '/', num2str(iterations)];
    title(title_text);
    hold off;
    
end

%% Function to display an histogram of successful gaits of each Kp during a Kp swept

function kp_optim (successful_vector_gains)

    figure
    histogram(successful_vector_gains, 11);
    hold on
    title('Kp successful walking gaits');
    hold off

end