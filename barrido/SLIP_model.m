function [sim_data,performance, result, errores]=SLIP_model(parameters, vel_objetivo, time,l_0, Kp)
% Function SLIP_model simulates SLIP model

% Input variables:
%%% parameters - set of SLIP model variable parameters
%%% profile - ground profile [x,y]
%%% time - maximum time to run simulation for

% dqdt variables:
%%% sim_data - motion data of model including coordinates.
%%% performance - struct with number of steps, time and distance before
%%% losing stability

%% Variable parameters
q=[parameters(1:4)];
alpha_firstleg=parameters(5)*pi/180;
alpha_secondleg=parameters(6)*pi/180;
K=parameters(7);
errores=[];

%% Fixed parameters
g = 9.81;		% acceleration due to gravity [m/s^2]
mass = 10/3;		% Point mass at hip [kg]
		% Uncompressed leg spring length [m]

x0 = 0;			% initial  coordinates of support leg, reference
y0 = 0;

timeremaining=time;
ie_double=0;
performance.terminationmsg='Not specified';

%% Transition Conditions for ending each phase

options_single = odeset('Events', @end_single_stance);
options_double = odeset('Events', @end_double_stance);
options_flight = odeset('Events', @end_flight);

%% Simulation
i=1;

while true
	% Start with single stance
	[t,q,~,~,ie_single] = ode45(@EoM_single, [0,timeremaining], q, options_single);
	sim_data(2*i-1).type='single';
	sim_data(2*i-1).support=[x0; y0];
	sim_data(2*i-1).t=t;
	sim_data(2*i-1).q=[q(:,1) + x0, q(:,2) + y0, q(:,3:4)];
    sim_data(2*i-1).T= Kp*(vel_objetivo-q(:,3));
    sim_data(2*i-1).error = (vel_objetivo-q(:,3));
    errores = [errores;sim_data(2*i-1).error];
	timeremaining=timeremaining-t(end);
    if timeremaining<10e-5; performance.terminationmsg=('Simulation time exceeded'); disp('Simulation time exceeded'); result = 1; break; end
	if ie_single==4;  performance.terminationmsg=('Body touched ground'); disp('Body touched the ground'); result = 0; break; end
	q = q(end,:);
    
    
	% Proceed to double support motion
	if ie_single==1
		x1 = q(1) + l_0*cos(alpha_secondleg); % Hor displacement between supports
		y1 = q(2) - l_0*sin(alpha_secondleg); % Ver displacement between supports
		[t,q,~,~,ie_double] = ode45(@EoM_double, [0,timeremaining], q, options_double);
		sim_data(2*i).type='double';
		sim_data(2*i).support=[x0, x0+x1; y0, y0+y1];
		sim_data(2*i).t=t;
		sim_data(2*i).q=[q(:,1) + x0,q(:,2) + y0, q(:,3:4)];
        sim_data(2*i).T= Kp*(vel_objetivo-q(:,3));
        sim_data(2*i).error = (vel_objetivo-q(:,3));
        errores = [errores;sim_data(2*i).error];
        if ie_double==2;performance.terminationmsg=('Backward motion started'); disp('Backward motion started'); result = 0; break; end
        timeremaining=timeremaining-t(end);
        
		if timeremaining<10e-5;performance.terminationmsg=('Simulation time exceeded');disp('Simulation time exceeded'); result = 1; break; end
		
		% Proceed to flight motion
	elseif ie_single==2
		[t,q,~,~,ie_flight] = ode45(@EoM_flight, [0,timeremaining], q,options_flight);
       	x1 = q(end,1) + l_0*cos(alpha_firstleg);   % Horizontal distance between consecutive supports
		y1 = q(end,2) - l_0*sin(alpha_firstleg);   % Vertical distance between consecutive supports
		sim_data(2*i).type='flight';
		sim_data(2*i).support=zeros(2,0);
		sim_data(2*i).t=t;
		sim_data(2*i).q=[q(:,1) + x0,q(:,2) + y0, q(:,3:4)];
        sim_data(2*i).T=zeros(length(q(:,1)),1);
        sim_data(2*i).error = (vel_objetivo-q(:,3));
        errores = [errores;sim_data(2*i).error];
		timeremaining=timeremaining-t(end);
        if timeremaining<10e-5; performance.terminationmsg=('Simulation time exceeded');disp('Simulation time exceeded'); result = 1; break; end
        if ie_flight==2; performance.terminationmsg=('Body touched ground'); disp('Body touched ground'); result = 0; break; end

		% Stop simulation
	elseif ie_single==3; performance.terminationmsg=('Backward motion started'); disp('Backward motion started'); result = 0; break
        else performance.terminationmsg=('Unknown termination cause');  disp('Unknown termination cause');result = 0; break; end
	i=i+1;
	
	if ie_double==3 % The leg which touched ground last took off first
		q=q(end,:);
		ie_double=0;
		continue
	end
	x0 = x0 + x1; % New support coordinates
	y0 = y0 + y1; %
    
         
	q = [q(end,1) - x1, q(end,2) - y1, q(end,3:4)];
    
   
end

%% Assess performance
performance.Time=0;
for j=1:length(sim_data)
	performance.Time=performance.Time+sim_data(j).t(end); % Time passed before losing stability
end

support=[sim_data.support];
performance.Steps=length(unique(support(1,:))); % Number of steps before losing stability

if length(sim_data)==1
	performance.Distance=0;
else
	performance.Distance=sim_data(end-1).q(end,1); % Distance covered before losing stability
end

%% Equations of motions:
% input  q=   [x ; y ; x' ; y' ]
% output dqdt=[x'; y'; x''; y'']
	% Flight phase 
	function dqdt = EoM_flight(~,q)
		dqdt(1,1) = q(3);
		dqdt(2,1) = q(4);
		dqdt(3,1) = 0;
		dqdt(4,1) = -g;
	end

	% Single stance phase
	function dqdt = EoM_single(~,q)
        %% Regulador con el PAR 
        T = Kp*(vel_objetivo-q(3)); 
		dqdt(1,1) = q(3);
		dqdt(2,1) = q(4);
		l1=sqrt(q(1)^2+q(2)^2); % spring length
		a1=K*(l_0-l1)/mass;		% acceleration of point mass
        a_par = T/(l1*mass);
		dqdt(3,1) = a1/l1*q(1) + a_par/l1*q(2);
		dqdt(4,1) = a1/l1*q(2) + a_par/l1*q(1) - g;
	end
	
	% Double stance phase
	function dqdt = EoM_double(~,q)
        %% Regulador con el PAR 
        
        T = Kp*(vel_objetivo-q(3)); 
    
        dqdt(1,1) = q(3);
		dqdt(2,1) = q(4);
		l1=sqrt(q(1)^2+q(2)^2);				% length of first spring
		l2=sqrt((q(1)-x1)^2+(q(2)-y1)^2);	% length of second spring
		a1=K*(l_0-l1)/mass;					% acceleration due to first spring
		a2=K*(l_0-l2)/mass;					% acceleration due to second spring
        a_par1 = T/(l1*mass);
        a_par2 = T/(l2*mass);
		dqdt(3,1) = a1*q(1)/l1 + a2*(q(1)-x1)/l2 + a_par1/l1*q(2) + a_par2/l2*(q(2)-x1);
		dqdt(4,1) = a1*q(2)/l1 + a2*(q(2)-y1)/l2 + a_par1/l1*q(1) + a_par2/l2*(q(1)-y1)- g;
	end

%% Event functions of equation of motions to detect phase change or termination
	function [impact,terminate,direction] = end_flight(~,q)
		% Event 1 - First leg touched ground
		% Event 2 - Body touched ground
		impact = [q(2) - l_0*sin(alpha_firstleg) + y0,...
			q(2)];
		terminate = [1, 1];
		direction = [-1,-1];
	end

% Single stance phase
	function [impact,terminate,direction] = end_single_stance(~,q)
		% Event 1 - Second leg touched ground
		% Event 2 - Leg length exceeded uncompressed spring length
		% Event 3 - Body started backward motion
		% Event 4 - Body touched ground
		impact = [q(2) - l_0*sin(alpha_secondleg) + y0 ,...
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
end