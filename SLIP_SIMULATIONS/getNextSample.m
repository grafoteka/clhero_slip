function bestx = getNextSample(gp, searchrange,y,x,maxexpectedy) %#ok
% Function getNextSample estimates the point where to sample next to
% improve the probability of improving the maximum value by at least a
% specified value
% Input parameters:
%	gp - RegressionGP variable
%	x - parameters of the existing sampled points
%	y - corresponding function values of the parameters
%	searchreange - range of parameters in which to

% Author: Kaur Aare Saar (kas82@cam.ac.uk), August 2016


ymax=max(y);
epsilon=10*exp(-length(y)/50); %maximise the probability to get improvement of at least epsilon
j=0;
lastbestPI=-inf;
N=(searchrange(:,1)~=searchrange(:,2));
n=round(1e4^(1/sum(N)));
N=1+N*(n-1);
disp('    Performing grid search:')
while true % perform iteretaive grid seach until specified accuracy has been achieved
	j=j+1;
	for i=1:length(searchrange)
		values{i}=linspace(searchrange(i,1),searchrange(i,2),N(i));
	end
	states=combvec(values{:})';
	states=[states;x];
	[mean,std]=predict(gp,states);
	%threshold=min(maxexpectedy,1.5*ymax);
	threshold=ymax+epsilon;
	PI=(mean-threshold)./std;	
	[maxPI,idx]=max(PI);
	bestx=states(idx,:);

	disp(['        Iteration number: ', num2str(j),' , Best probability of improvement: ',num2str(1/2*(1+erf(maxPI/sqrt(2))),'%.3g')])

	
	newrange(:,1)=max(searchrange(:,1),bestx'-(searchrange(:,2)-searchrange(:,1))./(N-1));
	newrange(:,2)=min(searchrange(:,2),bestx'+(searchrange(:,2)-searchrange(:,1))./(N-1));
	
	searchrange=newrange;
	if maxPI-lastbestPI<0.01 % required accuracy of the seach
		break
	end
	lastbestPI=maxPI;
end
end

