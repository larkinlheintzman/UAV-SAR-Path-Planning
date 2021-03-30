%%%%% Sept 3, 2020 - This version of the mobility model is smoothed with velocity and includes
%%%%% all DT and fills in NaN if any reps go off the map. Also includes the new map and inaccessible layers.
%%%%% DOES NOT include railroads, powerlines, inaccessible elevation gradient
clearvars; close all; clc
set(0,'DefaultFigureWindowStyle','normal');

% Import map data files from Tianzi - elev, water, roads, lakes
% fnameelev = 'location4/-80.75195014146442,37.282633563856,-80.33996283677747,37.12539860056254_1597340249908_elevation.csv';
% fnamew = 'location4/-80.75195014146442,37.282633563856,-80.33996283677747,37.12539860056254_1597340519498_nhd6_river.txt';
% fnamer = 'location4/-80.75195014146442,37.282633563856,-80.33996283677747,37.12539860056254_1597340684535_trans30_road.txt';
% fnamel = 'location4/-80.75195014146442,37.282633563856,-80.33996283677747,37.12539860056254_1597340805300_nhd8_lake.txt';
% [sZelev, BWLF, BWInac] = importmap(fnameelev,fnamew,fnamer,fnamel);
% close all
load('BW_LFandInac_Zelev.mat')

% limits of map
ll = 1;
LLx = size(BWLF,2);
LLy = size(BWLF,1);

% input first positions and convert them to body coordinates
iclonlat = [-80.5459564891223,37.2040570285143;...
    -80.4757389347337,37.2040570285143;...
    -80.5450562640660,37.1977538769385];
ics = convert_ics(iclonlat);


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% define possible motions in body coords
right = [1;0];
left = [-1;0];
front = [0;1];
back = [0;-1];
stay = [0;0];
frontRight = [1;1];
frontLeft = [-1;1];
backRight = [1;-1];
backLeft = [-1;-1];

% define possible motions
possibleMotions = [frontLeft, front, frontRight, left, stay, right, backLeft, back, backRight];

% load behavior distribution
load('beh_dist_6');

% smoothing parameter
alpha = 0.55;

% replicates
reps = 100;

% walking speed - ts time steps per hour
ts = 850;   % speed(6.67m/6s) * conversion(3600s/6.67m) = ts/hr

%% Run the simulation

% define the pdf from a normal dist for mobility in hours
Mobi = zeros(length(probs),reps,10);

% this is where we can choose the LP type - imob
Mobi(:,:,1) = mob_hiker(length(probs),reps)*ts;
Mobi(:,:,2) = mob_child10to12(length(probs),reps)*ts;
Mobi(:,:,3) = mob_child1to6(length(probs),reps)*ts;
Mobi(:,:,4) = mob_dementia(length(probs),reps)*ts;
Mobi(:,:,5) = mob_hunter(length(probs),reps)*ts;
Mobi(:,:,6) = mob_angler(length(probs),reps)*ts;
Mobi(:,:,7) = mob_child13to15(length(probs),reps)*ts;
Mobi(:,:,8) = mob_despond(length(probs),reps)*ts;
Mobi(:,:,9) = mob_snowboard(length(probs),reps)*ts;
Mobi(:,:,10) = mob_worker(length(probs),reps)*ts;
savename = {'hiker','child10to12','child1to6','dementia','hunter','angler','child13to15','despond','snowboard','worker'};

for imob = 1:10
    Mob = Mobi(:,:,imob);
    
    % initial BW_temp (0 or 1) for initial position
    BW_temp = nan(3,3);
    
    all_data = cell(length(probs),reps,size(ics,1)); % cell array where each entry is a T(variable) x 3 matrix     %%%%%%%%%%%%%%%%%%    
    
    for iic = 1:length(ics)
        
        for iprob = 1:length(probs)
            [imob iic iprob]
            
            for irep = 1:reps
                
                % define the behavior profile and length of simulation
                p_behavior = probs(iprob,:);
                T = ceil(Mob(iprob, irep));
                
                % input first two positions
                X(1) = ics(iic,1);
                Y(1) = ics(iic,2);
                X(2) = X(1)+(floor(3*rand)-1);
                Y(2) = Y(1)+(floor(3*rand)-1);
                
                % initialize position matrix
                x = nan(1,T+2); y = nan(1,T+2); behavior = nan(1,T+2);
                x(1) = X(1); y(1) = Y(1);
                x(2) = X(2); y(2) = Y(2);
                
                % initialize velocity and compute initial value in body coords
                u = nan(1,T+1); v = nan(1,T+1);
                u(1) = x(2)-x(1); v(1) = y(2)-y(1);
                
                % initialize vx and vy
                vx = nan(1,T+2); vy = nan(1,T+2);
                xs = nan(1,T+2); ys = nan(1,T+2);
                
                % initialize possible behavior matrix
                XY_temp = zeros(2,1);
                
                % initialize flag for going off the map
                flag = 0;
                
                for ii = 2:T+1
                    
                    %%% check if it went off the map, break out for a failed rep if it did
                    if flag == 1
                        break
                    end
                    
                    % choose motion based on type of person from p_behavior
                    behavior(ii) = pdf_sample(p_behavior,1);     
                    
                    % initialize staying put flag
                    flag_sp = 0;
                    
                    %%%%%%%%%%%%% COMPUTE ALL POSSIBLE BEHAVIORS %%%%%%%%%%%%%%
                    
                    if behavior(ii) == 1
                        %%%%%%%%%%%%%%%%%%%%%% 1. random traveling (rw) %%%%%%%%%%%%%%%%%%%%%%%%%%
                        
                        px_rw = [1, 1, 1, 1, 1, 1, 1, 1, 1]/9; % random walk pdf
                        inds = pdf_sample(px_rw, 1);
                        motions = possibleMotions(:,inds);
                        
                        % provisional update for random traveling
                        u = motions(1); v = motions(2);             % updated velocity in body coordinates
                        
                        if x(ii)-x(ii-1)==0 && y(ii)-y(ii-1) ==0    % prevents arctan2 from having (0/0)
                            theta = 2*pi*rand;
                        else
                            theta = -atan2(x(ii)-x(ii-1),y(ii)-y(ii-1));        % angle of previous velocity in global coordinates
                        end
                        
                        M = [cos(theta), -sin(theta), x(ii);...
                            sin(theta), cos(theta), y(ii);...
                            0, 0, 1];                               % transformation for rotation by theta and translation by previous position
                        
                        temp = M*[u; v; 1];                         % updated position in global coordinates computed from transformed updated velocity in body coords
                        x_rw = round(temp(1));
                        y_rw = round(temp(2));
                        
                        XY_temp = [x_rw y_rw];
                        
                    elseif behavior(ii) == 2
                        %%%%%%%%%%%%%%%%%%%%%% 2. route traveling (rt) %%%%%%%%%%%%%%%%%%%%%%%%%%
                        
                        px_rt = [3, 3, 3, 0, 0, 0, 0, 0, 0]/9;
                        
                        
                        if x(ii)-x(ii-1)==0 && y(ii)-y(ii-1) ==0    % prevents arctan2 from having (0/0)
                            theta = 2*pi*rand;
                        else
                            theta = -atan2(x(ii)-x(ii-1),y(ii)-y(ii-1));        % angle of previous velocity in global coordinates
                        end
                        
                        M = [cos(theta), -sin(theta), x(ii);...
                            sin(theta), cos(theta), y(ii);...
                            0, 0, 1];                               % transformation for rotation by theta and translation by previous position
                        
                        % create a new pdf for the prob of being on a linear feature using BW
                        for jj = 1:9
                            aux = possibleMotions(:,jj);            % choose a motion
                            aux1 = round(M*[aux(1); aux(2); 1]);    % put the updated position in world coordinates
                            aux1(end) = [];
                            
                            if aux1(2)>LLy || aux1(2)<ll || aux1(1)>LLx || aux1(1)<ll
                                flag = 1;
                                break
                            end
                            
                            BW_temp(jj) = BWLF(aux1(2),aux1(1));
                        end
                        
                        if flag == 1
                            break
                        end
                        
                        px_lin = px_rt'.*BW_temp(:);                % pdf for linear feature using random trav px_rt
                        px_lin = px_lin'/norm(px_lin,1);            % normalizing the probability
                        px_lin(isnan(px_lin))= 1/9;                 % if no linear features, dist will be NaN, so set it back to random walk but it doesn't get used****
                        inds_lin = pdf_sample(px_lin, 1);
                        motions_lin = possibleMotions(:,inds_lin);
                        
                        % provisional update for route traveling
                        u = motions_lin(1); v = motions_lin(2);     % updated velocity in body coordinates
                        temp_lin = M*[u; v; 1];                     % updated position in global coordinates computed from transformed updated velocity in body coords
                        x_rt = round(temp_lin(1));
                        y_rt = round(temp_lin(2));
                        
                        XY_temp = [x_rt y_rt];
                        
                    elseif behavior(ii) == 3
                        %%%%%%%%%%%%%%%%%%%%%% 3. direction traveling (dt) %%%%%%%%%%%%%%%%%%%%%%%%%%
                        
                        px_dt = [0, 9, 0, 0, 0, 0, 0, 0, 0]/9;      % pdf for traveling in forward direction
                        
                        inds = pdf_sample(px_dt, 1);
                        motions = possibleMotions(:,inds);
                        
                        u = motions(1); v = motions(2);             % updated velocity in body coordinates
                        if x(ii)-x(ii-1)==0 && y(ii)-y(ii-1) ==0    % prevents arctan2 from having (0/0)
                            theta = 2*pi*rand;
                        else
                            theta = -atan2(x(ii)-x(ii-1),y(ii)-y(ii-1));        % angle of previous velocity in global coordinates
                        end
                        
                        M = [cos(theta), -sin(theta), x(ii);...
                            sin(theta), cos(theta), y(ii);...
                            0, 0, 1];                               % transformation for rotation by theta and translation by previous position
                        
                        temp = M*[u; v; 1];                         % updated position in global coordinates computed from transformed updated velocity in body coords
                        x_dt = round(temp(1));
                        y_dt = round(temp(2));
                        
                        XY_temp = [x_dt y_dt];
                        
                    elseif behavior(ii) == 4
                        %%%%%%%%%%%%%%%%%%%%%% 4. staying put (sp) %%%%%%%%%%%%%%%%%%%%%%%%%%
                        
                        x_sp = x(ii);
                        y_sp = y(ii);
                        
                        flag_sp = 1;
                        
                        XY_temp = [x_sp y_sp];
                        
                    elseif behavior(ii) == 5
                        %%%%%%%%%%%%%%%%%%%%%% 5. view enhancing (ve) %%%%%%%%%%%%%%%%%%%%%%%%%%
                        if x(ii)-x(ii-1)==0 && y(ii)-y(ii-1) ==0    % prevents arctan2 from having (0/0)
                            theta = 2*pi*rand;
                        else
                            theta = -atan2(x(ii)-x(ii-1),y(ii)-y(ii-1));        % angle of previous velocity in global coordinates
                        end
                        
                        M = [cos(theta), -sin(theta), x(ii);...
                            sin(theta), cos(theta), y(ii);...
                            0, 0, 1];                               % transformation for rotation by theta and translation by previous position
                        
                        % create a new pdf based on intensity of gradient elevation (int)
                        for kk = 1:9
                            auxv = possibleMotions(:,kk);            % choose a motion
                            auxv1 = round(M*[auxv(1); auxv(2); 1]);    % put the updated position in world coordinates
                            auxv1(end) = [];
                            
                            % check if off the map
                            if auxv1(2)>LLy || auxv1(2)<ll || auxv1(1)>LLx || auxv1(1)<ll
                                flag = 1;
                                break
                            end
                            
                            int_temp(kk) = sZelev(auxv1(2),auxv1(1)); % find the elevation for each of the 9 possible movements
                        end
                        
                        if flag == 1
                            break
                        end
                        
                        int_temp = int_temp-int_temp(5); % subtract the elevation of "stay put", i.e. the elevation of the current position
                        
                        aux_max = max(int_temp);    % find the max elevation gain
                        inds_ve = find(int_temp==aux_max);  % find the position(s) where max occurs
                        inds_ve = inds_ve(randperm(length(inds_ve)));   % permute the indicies of those positions uniformly
                        motions_ve = possibleMotions(:,inds_ve(1)); % choose the first one as the updated position
                        
                        % provisional update for view enhancing
                        u = motions_ve(1); v = motions_ve(2); % updated velocity in body coordinates
                        temp_ve = M*[u; v; 1];                 % updated position in global coordinates computed from transformed updated velocity in body coords
                        x_ve = round(temp_ve(1));
                        y_ve = round(temp_ve(2));
                        
                        XY_temp = [x_ve y_ve];
                        
                    else
                        %%%%%%%%%%%%%%%%%%%%%% 6. backtracking (bt) %%%%%%%%%%%%%%%%%%%%%%%%%%
                        if behavior(ii-1) ~= 6          % if the last beh wasn't bt, go to the previous position
                            x_bt = x(ii-1);
                            y_bt = y(ii-1);
                        elseif behavior(ii-1) == 6      % if the last beh was bt, find the last non-bt (2 steps) and go to that position
                            BT_steps = 1;
                            while behavior(ii-BT_steps-1) == 6
                                BT_steps = BT_steps + 1;
                            end
                            ind_bt = max(ii - 2*(BT_steps)-1, 1);
                            x_bt = x(ind_bt);
                            y_bt = y(ind_bt);
                        else
                        end
                        XY_temp = [x_bt y_bt];
                        
                    end
                    
                    
                    %%%%%%%%%%% choose update from provisional updates %%%%%%%%%%
                    % smoothing with previous velocity
                    vx(ii) = XY_temp(1) - x(ii);
                    vy(ii) = XY_temp(2) - y(ii);
                    xs(ii+1) = round((2-alpha)*x(ii) + (alpha-1)*x(ii-1) + alpha*vx(ii));
                    ys(ii+1) = round((2-alpha)*y(ii) + (alpha-1)*y(ii-1) + alpha*vy(ii));
                    
                    x(ii+1) = (1-flag_sp)*xs(ii+1) + flag_sp*XY_temp(1);
                    y(ii+1) = (1-flag_sp)*ys(ii+1) + flag_sp*XY_temp(2);
                    
                    %%% check if it went off the map, break out for a failed rep if it did
                    if x(ii+1)>LLx || x(ii+1)<ll || y(ii+1)>LLy || y(ii+1)<ll
                        flag = 1;
                    end
                    %%% check if provisional update is inaccessible, stay put if it is
                    if flag == 0 && BWInac(y(ii+1),x(ii+1)) == 1
                        x(ii+1) = x(ii);
                        y(ii+1) = y(ii);
                    end
                    
                    
                end
                
                all_data{iprob,irep,iic} = [x' y' behavior'];
                
            end
            
        end
%         all_data_ic = all_data(:,:,iic);
%         save(['simulations/sim_ic',num2str(iic),'_',num2str(savename{imob}),'.mat'],'all_data_ic','probs','Mob','-v7.3')
%         clear all_data_ic
    end
    save(['data/all_data_',num2str(savename{imob}),'.mat'],'all_data','probs','Mob','-v7.3')
    clear all_data
end


%%
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
