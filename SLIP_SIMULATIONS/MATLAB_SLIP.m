function MATLAB_SLIP(vel_inicial, vel_objetivo,stiffness, angle_walking, angle_running, leg_size, Kp, Ki)


%% Set Initial Conditions and Search Ranges
% Vector InitialConditions defines the initial state as [x y x' y']
% Normally x=y'=0
compresion_inicial = leg_size - 10*9.8/stiffness;

%InitialConditions=[0	0.98	1.3	0]; % walking
%InitialConditions=[0	0.95	1.6	0]; % skipping
%InitialConditions=[0	0.95	5	0]; % running
InitialConditions= [0 compresion_inicial vel_inicial 0];

% Matrix searchrange defines minimum and maximum values for each parameter in
% which to seach for best performance
searchrange=[	
	angle_running angle_running;	% touchdown angle when running (deg)
	angle_walking angle_walking;		% touchdown angle when walking (deg)
	stiffness stiffness;	% spring constant (N/m)
	];  
names=[
	'Touchdown angle when running (deg): ';
	'Touchdown angle when walking (deg): ';
	'Spring stiffness (N/m):             ';
	];
labels={
	' \alpha_1 ( ^\circ )';
	' \alpha_2 ( ^\circ )';
	' k ( N / m )';
	};
parameter_count=size(searchrange,1);

%% Performance assessment and optimisation parameters
disturbance=0.01;			% Relative amplitude of Gaussian noise added to initial conditions
totaltests=1;				% Number of tests to average over for each parameter set
maxtime=15;				% Maximum time for simulation (s)
fitnessfunction='Distance';	% Fitness function to use as objective function 'Time', 'Steps' or 'Distance'
maxexpectedfitness=maxtime*InitialConditions(3);

profile=[0 0; 1000 0];		% Flat ground of length 1000 (m)
N=1;						% Number of iterations to run the optimisation for
KernelMode='ARDMatern32';				% Allow more rapid changes in the objective function
%KernelMode='ARDSquaredExponential';	% Force the objective function to be smoother


%% Choose random sample for first evaluation
x=((searchrange(:,2)-searchrange(:,1)).*rand(size(searchrange,1),1)+searchrange(:,1))'; 
y=[];


%% Iterate
for sample_no = length(y)+1:N
	% Evaluate function at
	parameters = x(end,:);
	disp(['Evaluation number ',num2str(sample_no)]);
	disp('    Parameters:');
	disp([repmat(' ',parameter_count,8),names,num2str(parameters','%-.3g')]);
	
	for test_no=1:totaltests
 		ICwithdisturbance=InitialConditions; 
		[data,performance(test_no)]=SLIP_MODEL_TORQUE([ICwithdisturbance,angle_running,angle_walking,stiffness],vel_objetivo, maxtime,leg_size, Kp, Ki); 
        
        if strcmp(performance.terminationmsg,'Simulation time exceeded')
           plotting(data,vel_objetivo, leg_size, angle_walking, angle_running,Kp, Ki);
%          animation(data,profile);
%            print(gcf,'hola','-dpng','-r500');
        end

	end
	disp(['    Average performance over ',num2str(totaltests),' tests:'])
	disp(['        ','Distance:         ',num2str(mean([performance.Distance]),'%.3g'),' ± ',num2str(std([performance.Distance]),'%.3g'), ' (m)'])
	disp(['        ','Time:             ',num2str(mean([performance.Time    ]),'%.3g'),' ± ',num2str(std([performance.Time    ]),'%.3g'), ' (s)'])
	disp(['        ','Steps:            ',num2str(mean([performance.Steps   ]),'%.3g'),' ± ',num2str(std([performance.Steps   ]),'%.3g')])
	fitness=mean([performance.(fitnessfunction)]); % Use the mean of the fitnesses over all tests for given parameters
	
	y = [y; fitness];

	
	% Update GP
	if sample_no==N; break; end
	disp('    Updating Gaussian process')
	gp=fitrgp(x,y,'KernelFunction',KernelMode); % introduced in MATLAB 2015b 
	
	% Find new sampling point
	xNew = getNextSample(gp,searchrange,y,x,maxexpectedfitness);
	x = [x; xNew]; 
   end
end
