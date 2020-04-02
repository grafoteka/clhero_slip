%%=====================================================================

%Authors: Jorge de Le�n Rivas and Rodrigo S�nchez Molina

%File: 'SLIP_model_walking.m'

%Version: Beta 1.0

%Description: %Function to solve TD-SLIP template equations and alternate among single
%stance phase, double stance phase and flight phase. 

%Changelog:

%%=====================================================================

function [data,performance, result]=SLIP_model_walking(initial_parameters, desired_vel, time,l_0, Kp)


% Initial_parameters = [x_initial, y_initial, vx_initial, vy_initial, angle_walking, stiffness, mass]

q = initial_parameters(1:4); % m || m/s
angle_walking = initial_parameters(5)*pi/180; %rad
angle_running = NaN;
K = initial_parameters(6); % N/m
mass = initial_parameters(7); % Kg



%% Reference initial conditions 
g = 9.81; % gravity m/s^2
x0 = 0;			
y0 = 0;

timeremaining = time;
result_double = 0;
performance.terminationmsg = 'Not specified';

%% Odeset transitions to alternate among phases

config_single = odeset('Events', @end_single_stance);
config_double = odeset('Events', @end_double_stance);
config_flight = odeset('Events', @end_flight);

%% Simulation
counter=1;



while true
    
	% start with single stance
	[t,q,~,~,result_single] = ode45(@EoM_single, [0,timeremaining], q, config_single); %solving single equations
    
    % q is now a matrix with [x,y,vx,vy] in its columns
    
    % storing data
	data(2*counter-1).type = 'single'; 
	data(2*counter-1).support = [x0; y0]; % supporting point to the ground
	data(2*counter-1).t = t; % time from beginning to ending of this phase
	data(2*counter-1).q = [q(:,1) + x0, q(:,2) + y0, q(:,3:4)]; % [x,y,vx,vy]
    data(2*counter-1).T = Kp*(desired_vel-q(:,3)); % torque N/m
    [data(2*counter-1).Fx, data(2*counter-1).Fy] = forces_single(q); % Resultant forces in the CoM of the robot
    
    %updating remaining time
	timeremaining=timeremaining-t(end);
    
    % check if the simulation time is achieved
    if timeremaining < 10e-5; performance.terminationmsg=('Simulation time achieved'); disp('Simulation time achieved'); result = 1; break; end
    
    %check if the CoM has touched ground
	if result_single==4;  performance.terminationmsg=('Body touched ground'); disp('Body touched the ground'); result = 0; break; end
    
    % update the initial conditions
    q = q(end,:);
    
    
	% check if passed to double stance phase
	if result_single==1
        
		x1 = q(1) + l_0*cos(angle_walking); % new x support point for the second leg
		y1 = q(2) - l_0*sin(angle_walking); % new y support point for the second leg
        
        
		[t,q,~,~,result_double] = ode45(@EoM_double, [0,timeremaining], q, config_double);
        
        % storing data
		data(2*counter).type = 'double';
		data(2*counter).support = [x0, x0+x1; y0, y0+y1];
		data(2*counter).t = t;
		data(2*counter).q = [q(:,1) + x0,q(:,2) + y0, q(:,3:4)];
        data(2*counter).T = Kp*(desired_vel-q(:,3));
        [data(2*counter).Fx,data(2*counter).Fy] = forces_double(q);
        
        %updating remaining time
        timeremaining = timeremaining-t(end);

        % checking if backward motion started
        if result_double == 2; performance.terminationmsg=('Backward motion started'); disp('Backward motion started'); result = 0; break; end
        

        %checking if simulation time is achieved
		if timeremaining < 10e-5; performance.terminationmsg=('Simulation time achieved');disp('Simulation time achieved'); result = 1; break; end
		
	% checking if passed to flight phase
	elseif result_single == 2; performance.terminationmsg=('Body flight'); disp('Body flight'); result = 0; break;  
		
%         [t,q,~,~,result_flight] = ode45(@EoM_flight, [0,timeremaining], q,config_flight);
% 
%         %calculating new support points for the entering leg
%         x1 = q(end,1) + l_0*cos(alpha_firstleg);   % Horizontal distance between consecutive supports
%         y1 = q(end,2) - l_0*sin(alpha_firstleg);   % Vertical distance between consecutive supports
%         sim_data(2*counter).type='flight';
%         sim_data(2*counter).support=zeros(2,0);
%         sim_data(2*counter).t=t;
%         sim_data(2*counter).q=[q(:,1) + x0,q(:,2) + y0, q(:,3:4)];
%         sim_data(2*counter).T=zeros(length(q(:,1)),1);
%         sim_data(2*counter).error = (desired_vel-q(:,3));
%         [sim_data(2*counter).Fx, sim_data(2*counter).Fy]=forces_flight(q);
%         
%         % updating timeremaining
%         timeremaining=timeremaining-t(end);
%         
%         %check if simulation time is achieved
%         if timeremaining<10e-5; performance.terminationmsg=('Simulation time achieved');disp('Simulation time achieved'); result = 1; break; end
%         
%         %check if CoM has touched ground
%         if result_flight==2; performance.terminationmsg=('Body touched ground'); disp('Body touched ground'); result = 0; break; end


	% check if backward motion started during single stance phase
    elseif result_single == 3; performance.terminationmsg=('Backward motion started'); disp('Backward motion started'); result = 0; break; 
    
    % unknown termination cause ocurred
    else; performance.terminationmsg=('Unknown termination cause'); disp('Unknown termination cause');result = 0; break; end;
    
    
   
	counter = counter + 1;
	
	if result_double == 3 % The leg which touched ground last took off first
		q=q(end,:); % update initial conditions
		result_double=0;
        continue;
    end
    
    % update new support reference
	x0 = x0 + x1; % m
	y0 = y0 + y1; % m
    
    %update initial conditions     
	q = [q(end,1) - x1, q(end,2) - y1, q(end,3:4)];
    
   
end

%% Update performance information

performance.Time=0;

for j=1:length(data)
	performance.Time=performance.Time+data(j).t(end); % Time passed before losing stability
end

support=[data.support];

performance.Steps=length(unique(support(1,:))); % Number of steps before losing stability

if length(data)==1
	performance.Distance=0;
else
	performance.Distance=data(end-1).q(end,1); % Distance covered before losing stability
end

%% Equations of motions:
% input  q=   [x ; y ; x' ; y' ]
% output dqdt=[x'; y'; x''; y'']

% 	% flight phase equations of motion EoM 
% 	function dqdt = EoM_flight(~,q)
% 		dqdt(1,1) = q(3);
% 		dqdt(2,1) = q(4);
% 		dqdt(3,1) = 0;
% 		dqdt(4,1) = -g;
% 	end

	% single stance phase equations of motion EoM
	function dqdt = EoM_single(~,q)

        T = Kp*(desired_vel-q(3)); % torque controller
		dqdt(1,1) = q(3); % vx
		dqdt(2,1) = q(4); % vy
		leg_1=sqrt(q(1)^2+q(2)^2); % instantaneous lenght of the leg
		a_spring=K*(l_0-leg_1)/mass; % acceleration due to the spring
        a_torque = T/(leg_1*mass); % acceleration due to the torque
		dqdt(3,1) = a_spring/leg_1*q(1) + a_torque/leg_1*q(2); % ax
		dqdt(4,1) = a_spring/leg_1*q(2) - a_torque/leg_1*q(1) - g; % ay
	end
	
	% Double stance phase
	function dqdt = EoM_double(~,q)
        %% Regulador con el PAR 
        
        T = Kp*(desired_vel-q(3)); 
    
        dqdt(1,1) = q(3); % vx
		dqdt(2,1) = q(4); % vy
		leg_1=sqrt(q(1)^2+q(2)^2);				% instantaneous lenght of the leg1
		leg_2=sqrt((q(1)-x1)^2+(q(2)-y1)^2);	% instantaneous lenght of the leg2
		a_first_spring=K*(l_0-leg_1)/mass;					% acceleration due to first spring
		a_second_spring=K*(l_0-leg_2)/mass;					% acceleration due to second spring
        a_first_torque = T/(leg_1*mass); % acceleration due to the torque1
        a_second_torque = T/(leg_2*mass); % acceleration due to the torque2
		dqdt(3,1) = a_first_spring*q(1)/leg_1 + a_second_spring*(q(1)-x1)/leg_2 + a_first_torque/leg_1*q(2) + a_second_torque/leg_2*(q(2)-y1); % ax
		dqdt(4,1) = a_first_spring*q(2)/leg_1 + a_second_spring*(q(2)-y1)/leg_2 - a_first_torque/leg_1*q(1) - a_second_torque/leg_2*(q(1)-x1) - g; % ay
	end

%% Event functions of equation of motions to detect transitions and termination

% 	function [impact,terminate,direction] = end_flight(~,q)
% 		% Event 1 - First leg touched ground
% 		% Event 2 - Body touched ground
% 		impact = [q(2) - l_0*sin(angle_running) + y0,...
% 			q(2)];
% 		terminate = [1, 1];
% 		direction = [-1,-1];
% 	end

% Single stance phase
	function [impact,terminate,direction] = end_single_stance(~,q)
		% Event 1 - Second leg touched ground
		% Event 2 - Leg length exceeded uncompressed spring length
		% Event 3 - Body started backward motion
		% Event 4 - Body touched ground
		impact = [q(2) - l_0*sin(angle_walking) + y0 ,...
			l_0^2 - q(1)^2 - q(2)^2,...
			q(3),...
			q(2)+y0];
		terminate = [ 1, 1, 1, 1];
		direction = [-1,-1,-1,-1];
	end

% Double stance phase
	function [impact,terminate,direction] = end_double_stance(~,q)
		% Event 1 - Leg that touched ground first took off ground
		% Event 2 - Body started backward motion
		% Event 3 - Leg that touched ground last took off ground
		impact = [q(1)^2 + q(2)^2 - l_0^2,...
			q(3),...
			(q(1)-x1)^2 + (q(2)-y1)^2 - l_0^2];
		terminate = [1 1 1];
		direction = [1 -1 1];
    end

    function [Fx, Fy] = forces_single(q)

        vel=repmat(desired_vel, [1,length(q(:,3))] )';
        l_original=repmat(l_0,  [1,length(q(:,3))])';
        gravity=repmat(g, [1,length(q(:,3))])';
        
        T = Kp.*(vel-q(:,3)); 

		l1=sqrt(q(:,1).^2+q(:,2).^2); % spring length
		a1=K.*(l_original-l1)./mass;		% acceleration of point mass
        a_par = T./(l1.*mass);
		Fx =mass*( a1./l1.*q(:,1) + a_par./l1.*q(:,2));
		Fy =mass*( a1./l1.*q(:,2) - a_par./l1.*q(:,1) - gravity);
    end

    function [Fx, Fy] = forces_double(q)

        vel=repmat(desired_vel, [1,length(q(:,3))] )';
        l_original=repmat(l_0,  [1,length(q(:,3))])';
        gravity=repmat(g, [1,length(q(:,3))])';
        x1_vector=repmat(x1, [1,length(q(:,3))])';
        y1_vector=repmat(y1, [1,length(q(:,3))])';
        T = Kp.*(vel-q(:,3)); 
    
		l1=sqrt((q(:,1)).^2+(q(:,2)).^2);				% length of first spring
		l2=sqrt((q(:,1)-x1_vector).^2+(q(:,2)-y1_vector).^2);	% length of second spring
		a1=K.*(l_original-l1)./mass;					% acceleration due to first spring
		a2=K.*(l_original-l2)./mass;					% acceleration due to second spring
        a_par1 = T./(l1.*mass);
        a_par2 = T./(l2.*mass);
		Fx = mass*( a1.*q(:,1)./l1 + a2.*(q(:,1)-x1_vector)./l2 + a_par1.*q(:,2)./l1 + a_par2.*(q(:,2)-y1_vector)./l2 );
		Fy = mass*( a1.*q(:,2)./l1 + a2.*(q(:,2)-y1_vector)./l2 - a_par1.*q(:,1)./l1 - a_par2.*(q(:,1)-x1_vector)./l2 - gravity);
    end

%     function [Fx, Fy] = forces_flight(q)
%         gravity=repmat(g, [1,length(q(:,3))])';
%         Fx = 0;
% 		Fy = -mass.*gravity;
%     end
end