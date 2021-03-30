function rs = pdf_sample(px, sampleSize)

% this function computes a random sample from a discrete pdf

% INPUT:
% px: discrete pdf given as a row vector
% sampleSize:L how many samples you want
% OUTPUT:
% rs: random sample of size sampleSize from distribution px 

% define discrete pdf (9 states but start with a 0 so cdf is right)
px = [0, px];
% compute cumulative summation for cdf
cdf = cumsum(px);
% set 0 as min
cdf = cdf - min(cdf);
% set 1 as max
cdf = cdf/max(cdf);
% get unif distributed random sample
rnd = rand(sampleSize, 1);
% initialize output
rs = zeros(size(rnd));
% check which state of the RV the random sample should be in according to
% cdf
for ii= 1:length(cdf)
    rs(rnd<cdf(length(cdf)-ii+1)) = length(cdf)-ii;
end

end






