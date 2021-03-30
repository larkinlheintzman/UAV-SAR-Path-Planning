% generates contour maps for each time step for the interface - new map
% 9/3/20
clearvars; close all; clc;
tic

% load the LPTs and their distance from IPP ring data
load('Dh_km.mat')
filename = hdist.LP; 

addpath("C:\Users\Larkin\planning_llh_bgc\LP model")
parameters;

% choose the LPTs to run {'hiker';'child10to12';'child1to6';'dementia';'hunter';'angler';'child13to15';'despond';'snowboard';'worker'};
% lpt = [1 2 4];  % hiker, child10to12, dementia
lpt = [1];  % hiker, child10to12, dementia

for iLP = 1:length(lpt)
    LP = lpt(iLP);
    
    % load the LPT type
    load(['../data/datachop_',filename{LP},'.mat'])
    
    % load best probability dist fit
    load(['../data/fits_',filename{LP},'.mat'])
    
    for bf = 1:3 %%%% choose the top best fitting behaviors
        
        %%%%% choose the ic to run (1, 2, 3)
        ic = 1;
        % load initial conditions and map dimensions conversion (xcrds,ycrds)
        load('allics_conversion.mat')
%         ics = icsxy(ic,:); 
        
        %%%%% find the position at each hour using the best fit probability dist and one ic
        % distance from IPP for 3 LPTs (75% ring)
%         Dh = hdist.Dh(LP,3);
        Dh = hdist.Dh(5,3); % limited by 10km extent
        LPT_bestfit = fits(bf,ic);
        
        % find the last position of the LP for each of the best fitting probs
        iprob = LPT_bestfit;
        hr = 600;               % each hour is 600 time steps
        hrs = 18;
        reps = 100;
        p = zeros(reps,2,hrs);
        
        % initial conditions - first position for each replicate
        for irep = 1:reps
            x = all_data{iprob,irep,ic}(:,1);
            y = all_data{iprob,irep,ic}(:,2);
            p(irep,:,1) = [x(1) y(1)];
        end
        % position of LP (x,y) for each rep and ic at different hours
        for it = 1:hrs
            time = hr*it;
            for irep = 1:reps
                x = all_data{iprob,irep,ic}(:,1);
                y = all_data{iprob,irep,ic}(:,2);
                if time > length(x)
                    p(irep,:,it+1) = [x(length(x)) y(length(x))];
                else
                    p(irep,:,it+1) = [x(time) y(time)];
                end
            end
        end
        
        %% Kernel Density Estimator
        
        % Each step in position corresponds to 6.7 meters or ((deg2km(.015)/250)*1000)
        % D_h rings are converted to steps by dividing by 0.0067 km
        st = deg2km(.015)/250;
        Dh = round(Dh/st);
        
        % define the grid that kernel density will be estimated over (75% ring)
        map75 = [ics(1)-Dh, ics(2)-Dh; ics(1)+Dh, ics(2)+Dh];
        int = 8;    % intervals for the map, so each step is about 50m
        [X,Y] = meshgrid(map75(1,1):int:map75(2,1),map75(1,2):int:map75(2,2));
        pts = [X(:),Y(:)];
        
        % bandwidth for kernel density estimation
        bw = 30;
        
        for tt = 2:hrs+1
            xt = p(:,1,tt);
            yt = p(:,2,tt);
            pos = [xt yt];
            
            % kernel density estimation
            [f,xi] = ksdensity(pos,pts,'Bandwidth',bw);
            timemat = repmat(tt-1,length(xi),1);
            fnorm = f/norm(f,1);
            sim_data = [timemat xi fnorm];
            
            % contour plot
            fplot = reshape(fnorm,size(X));
            figure(tt)
            C = contourf(X,Y,fplot,10);
            hold on
            [xc,yc,zc] = C2xyz(C); % extract coordinates of vertices
            
            % extract levels & the coordinates of the vertices (x,y) at each level
            con_data = [];
            for ii = 1:length(zc) % [levels, x, y]
                conaux = [repmat(zc(ii),length(xc{1,ii}),1),xc{1,ii}', yc{1,ii}'];
                con_data = [con_data; conaux];
                plot(conaux(:,2),conaux(:,3),'r')
            end
            hold on
            plot(xt,yt,'ko')
            plot(ics(1),ics(2),'kx')
            hold off
            
            % convert to latitude and longitude
            % contour_levlonlat = [con_data(:,1), xcrds(round(con_data(:,2)-pts(1,1))+1)', ycrds(round(con_data(:,3)-pts(1,2))+1)'];
%             contour_levlonlat = [con_data(:,1), xcrds(round(con_data(:,2))+1)', ycrds(round(con_data(:,3))+1)'];
%             points_tlonlatprob = [sim_data(:,1), xcrds(sim_data(:,2))', ycrds(sim_data(:,3))',sim_data(:,4)];
            
            % write to csv file to match what is expected on python side
            fdata = reshape(fplot, numel(fplot), 1); % only data of contour plot
            write_mat = [iclatlon(1); iclatlon(2); fdata];
            writematrix(write_mat, ['outputs/ic_',num2str(ic),'_con_', filename{LP}, '_t',num2str(tt),'.csv']);
            
%             % write data to .csv file
%             writematrix(contour_levlonlat,['ic',num2str(ic),'/',filename,'/best fits/contour data csv/con_',filename,'bf',num2str(bf),'_t',num2str(tt-1),'.csv'])
%             writematrix(points_tlonlatprob,['ic',num2str(ic),'/',filename,'/best fits/point data csv/pt_',filename,'bf',num2str(bf),'_t',num2str(tt-1),'.csv'])
%             
%             % save data
%             save(['ic',num2str(ic),'/',filename,'/best fits/all_',filename,'bf',num2str(bf),'_t',num2str(tt-1),'.mat'],'points_tlonlatprob','contour_levlonlat','-v7.3')
            
%             clearvars -except tt hrs p bw pts X Y xcrds ycrds filename bf all_data fits LP ics ic lpt
            
        end
        
        
    end
end
toc

