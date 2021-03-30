function x = randsmpl(p)

p = [0, p]; % 0 indexing fix, stupid matlab
cdf = cumsum(p); % get cdf
cdf = cdf/max(cdf); % normalize

x = find(rand > cdf,1,'last'); % from uniform 0-1 rv, find last bin it fits in, return index


end