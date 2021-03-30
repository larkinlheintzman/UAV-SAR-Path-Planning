%%%%% Sept 3, 2020 - This version of the mobility model is smoothed with velocity and includes
%%%%% all DT and fills in NaN if any reps go off the map. Also includes the new map and inaccessible layers.
%%%%% DOES NOT include railroads, powerlines, inaccessible elevation gradient
clearvars; close all; clc
set(0,'DefaultFigureWindowStyle','normal');

% % Import map data files from Tianzi - elev, water, roads, lakes
% location_folder = 'location4';
% % addpath(location_folder)
% elv_file = dir(strjoin({location_folder,'\*elevation*'},''));
% fnameelev = strjoin({location_folder,'/', elv_file.name},'');
% river_file = dir(strjoin({location_folder,'\*nhd6_river*'},''));
% fnamew = strjoin({location_folder,'/', river_file.name},'');
% road_file = dir(strjoin({location_folder,'\*trans30_road*'},''));
% fnamer = strjoin({location_folder,'/', road_file.name},'');
% lake_file = dir(strjoin({location_folder,'\*nhd8_lake*'},''));
% fnamel = strjoin({location_folder,'/', lake_file.name},'');
% % fnameelev = 'locationBozoo/-80.97706210131685,37.51929369135401,-80.76729189868041,37.432650195981815_1600120539492_elevation.csv';
% % fnamew = 'locationBozoo/-80.97706210131685,37.51748964322839,-80.76729189868041,37.43445629323058_1600120553154_nhd6_river.txt';
% % fnamer = 'locationBozoo/-80.97706210131685,37.51748964322839,-80.76729189868041,37.43445629323058_1600120550979_trans30_road.txt';
% % fnamel = 'locationBozoo/-80.97706210131685,37.51929369135401,-80.76729189868041,37.432650195981815_1600120550665_nhd8_lake.txt';
% [sZelev, BWLF, BWInac] = importmap(fnameelev,fnamew,fnamer,fnamel);
% close all
map_data = load('BW_LFandInac_Zelev_hmpark.mat');

% limits of map
ll = 1;
LLx = size(map_data.BWLF,2);
LLy = size(map_data.BWLF,1);
LL = [LLx, LLy, ll];


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% load behavior distribution
load('beh_dist_6');

parameters; % load reps, N, ics_num, and lpt

if any(map_data.BWInac(ics(:,2), ics(:,1))) % if any of these are 1's, a replicate wont be able to move
    fprintf("Inaccessible terrain on initial planning point\n");
end

total_time = 0;
counter = 1;

%% Run the simulation

% define the pdf from a normal dist for mobility in hours
Mobi = zeros(length(probs),reps,lpts);

% this is where we can choose the LP type - imob
% Mobi(:,:,1) = mob_hiker(length(probs),reps)*ts;
% Mobi(:,:,2) = mob_child10to12(length(probs),reps)*ts;
% Mobi(:,:,3) = mob_child1to6(length(probs),reps)*ts;
% Mobi(:,:,4) = mob_dementia(length(probs),reps)*ts;
% Mobi(:,:,5) = mob_hunter(length(probs),reps)*ts;
% Mobi(:,:,6) = mob_angler(length(probs),reps)*ts;
% Mobi(:,:,7) = mob_child13to15(length(probs),reps)*ts;
% Mobi(:,:,8) = mob_despond(length(probs),reps)*ts;
% Mobi(:,:,9) = mob_snowboard(length(probs),reps)*ts;
% Mobi(:,:,10) = mob_worker(length(probs),reps)*ts;

% LLH custom lost person selection
Mobi(:,:,1) = mob_hiker(length(probs),reps)*ts;
% Mobi(:,:,2) = mob_hunter(length(probs),reps)*ts;
% Mobi(:,:,3) = mob_angler(length(probs),reps)*ts;
% Mobi(:,:,4) = mob_snowboard(length(probs),reps)*ts;

M = size(Mobi,3);
I = ics_num;
P = length(probs);
total_iter = M*I*P;

for imob = 1:size(Mobi,3)
    Mob = Mobi(:,:,imob);
    
    % initial BW_temp (0 or 1) for initial position
    BW_temp = nan(3,3);
    
    all_data = cell(length(probs),reps,size(ics,1)); % cell array where each entry is a T(variable) x 3 matrix     %%%%%%%%%%%%%%%%%%    
    
    for iic = 1:size(ics,1)
        
        for iprob = 1:length(probs)
%             [imob iic iprob]
            if iprob ~= 1
                counter = counter + 1;
                total_time = total_time + toc; % running sum
                if mod(iprob, 10) == 0
                    fprintf("est. %0.2f ish minutes remaining\n", ((total_iter - counter)*(total_time/counter))/60)
                end
            end
            tic;
            % define the behavior profile and length of simulation
            p_behavior = probs(iprob,:);
            
            parfor irep = 1:reps
                
                T = ceil(Mob(iprob, irep));
                initial_point = ics(iic,:); % initial starting point
                
                
                [x, y, behavior] = run_replicate(initial_point, map_data, T, p_behavior, alpha, LL);
                
                all_data{iprob,irep,iic} = [x, y, behavior,];
                
            end
            
        end
%         all_data_ic = all_data(:,:,iic);
%         save(['simulations/sim_ic',num2str(iic),'_',num2str(savename{imob}),'.mat'],'all_data_ic','probs','Mob','-v7.3')
%         clear all_data_ic
    end
    save(['C:\Users\Larkin\planning_llh_bgc\LP model\data\all_data_',num2str(savename{imob}),'.mat'],'all_data','probs','Mob','-v7.3')
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