function Mob = mob_despond(P,R)
% this code is to find the mobilities (max time that a lost person stays
% mobile) for the lost person model

p = [0.25 0.5 0.75 0.95];   % percentages from LPB book
mob = [0 2 5 20];          % mobilities for despondent LPT from LPB book. These define a cdf for the mobility 

% compute pdf from cdf
bins = 0.5:1:20;                  % define bins to ninterpolate cdf
cdf = interp1(mob,p, bins);      % interpolate cdf at bins
% pdf = diff(cdf)/sum(diff(cdf));  % differentiate cdf to get pdf
% bins_pdf = 0.5:1:19.5;          % define bins for pdf (centers from cdf bins) 

% from plotting bins (x) versus cdf (y) in the curve fitting tool box, we saw that
% the cdf can be fit with a Weibull distribution. We fit the curve with a function of the form
% 1-exp(-(x/a)^b). By fitting the cdf, we are finding the best pdf that
% fits the distribution from the book. 
% The best lambda (R^2=0.9892) is below. 

a = 3.519;            % parameter of best fit Weibull distribution for pdf
b = 0.5464;
Mob = wblrnd(a,b,[P,R]);   % sample from that Weibull distribution, to get mobilities in hours
