function [x, y, behavior] = run_replicate(initial_point, map_data, T, p_behavior, alpha, LL)
% fully run a single replicate for the LP model, returns final x y location and behavior used


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

% input first two positions
% X(1) = ics(iic,1);
% Y(1) = ics(iic,2);
X(1) = initial_point(1);
Y(1) = initial_point(2);
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
% XY_temp = zeros(2,1);

% initialize flag for going off the map
flag = 0;

for ii = 2:T+1
    
    %%% check if it went off the map, break out for a failed rep if it did
    if flag == 1
        break
    end
    
    % choose motion based on type of person from p_behavior
    % behavior(ii) = pdf_sample(p_behavior,1);
    behavior(ii) = randsmpl(p_behavior);
    
    % initialize staying put flag
    flag_sp = 0;
    
    %%%%%%%%%%%%% COMPUTE ALL POSSIBLE BEHAVIORS %%%%%%%%%%%%%%
    
    if behavior(ii) == 1
        %%%%%%%%%%%%%%%%%%%%%% 1. random traveling (rw) %%%%%%%%%%%%%%%%%%%%%%%%%%
        
        px_rw = [1, 1, 1, 1, 1, 1, 1, 1, 1]/9; % random walk pdf
        % inds = pdf_sample(px_rw, 1);
        inds = randsmpl(px_rw);
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
            
            if aux1(2)>LL(2) || aux1(2)<LL(3) || aux1(1)>LL(1) || aux1(1)<LL(3)
                flag = 1;
                break
            end
            
            BW_temp(jj) = map_data.BWLF(aux1(2),aux1(1));
        end
        
        if flag == 1
            break
        end
        
        px_lin = px_rt'.*BW_temp(:);                % pdf for linear feature using random trav px_rt
        px_lin = px_lin'/norm(px_lin,1);            % normalizing the probability
        px_lin(isnan(px_lin))= 1/9;                 % if no linear features, dist will be NaN, so set it back to random walk but it doesn't get used****
        % inds_lin = pdf_sample(px_lin, 1);
        inds_lin = randsmpl(px_lin);
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
        
        % inds = pdf_sample(px_dt, 1);
        inds = randsmpl(px_dt);
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
        int_temp = zeros(9,1);
        for kk = 1:9
            auxv = possibleMotions(:,kk);            % choose a motion
            auxv1 = round(M*[auxv(1); auxv(2); 1]);    % put the updated position in world coordinates
            auxv1(end) = [];
            
            % check if off the map
            if auxv1(2)>LL(2) || auxv1(2)<LL(3) || auxv1(1)>LL(1) || auxv1(1)<LL(3)
                flag = 1;
                break
            end
            
            int_temp(kk) = map_data.sZelev(auxv1(2),auxv1(1)); % find the elevation for each of the 9 possible movements
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
    if x(ii+1)>LL(1) || x(ii+1)<LL(3) || y(ii+1)>LL(2) || y(ii+1)<LL(3)
        flag = 1;
    end
    %%% check if provisional update is inaccessible, stay put if it is
    if flag == 0 && map_data.BWInac(y(ii+1),x(ii+1)) == 1
        x(ii+1) = x(ii);
        y(ii+1) = y(ii);
    end
    
    
end

x = x';
y = y';
behavior = behavior';

end