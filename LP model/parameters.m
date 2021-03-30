% behavior combos
N = 462;
% replicates
reps = 100;

% choose the LPTs to run {'hiker';'child10to12';'child1to6';'dementia';'hunter';'angler';'child13to15';'despond';'snowboard';'worker'};
lpt = [1];  % hiker, hunter, angler, snowboard

% number of lost person types
lpts = numel(lpt);

% smoothing parameter
alpha = 0.55;

% walking speed - ts time steps per hour
ts = 850;   % speed(6.67m/6s) * conversion(3600s/6.67m) = ts/hr

% name list
savename = {'hiker'};
% full list below
% savename = {'hiker','hunter','angler','snowboard','worker','child10to12','child1to6','dementia','despond','child13to15'};


% input first positions and convert them to body coordinates
% iclatlon = [37.476914, -80.865864]; % bozoo
% iclatlon = [37.197730, -80.585233]; % kentland
iclatlon = [36.891640, -81.524214]; % hmpark
% iclatlon = [37.476276, -80.870257]; % inac bozoo



% iclatlon = [36.657110, -81.543333]; % mt rogers
% ;...
%             37.204057, -80.475738;... % originals
%             37.197736, -80.548285];

% initial points
ics_num = size(iclatlon,1);

iclonlat = fliplr(iclatlon);
        
% ics = convert_ics(iclonlat);
ics = [750, 750];
