%%%% PART1: Extracts the failed reps where agent left the map, chops saved data to exclude those reps 
%%%% PART2: Calculates the distance from the initial planning point (IPP) for only horizontal distances

clearvars; close all; clc
tic
load('Dh_km.mat')
load('beh_dist_6.mat')

addpath("C:\Users\Larkin\planning_llh_bgc\LP model")

parameters;

filename = hdist.LP;
for aa = 1:1 % just doing hiker
    load(['../data/all_data_',filename{aa},'.mat'],'all_data')
    
    %%%%% Extract all the replicates where simulation failed and the LP moved off the
    %%%%% map for all three ICs. Badrep is a 462x3 cell of bad reps for 3 ICS
    a = all_data;
%     reps = 100;
    ic = 1; % which ic to run
    badrep = cell(length(probs),ic);
    for ic = 1:ic
        for iprob = 1:length(probs)
            for irep = 1:reps
                if isnan(a{iprob,irep,ic}(end,1))
                    aux = irep;
                    badrep{iprob,ic} = [badrep{iprob,ic}; aux];
                else
                    
                end
            end
        end
    end
    times = badrep;
    
    for tt = 1:462
        if isempty(times{tt,1}) == 1 %&& isempty(times{tt,2}) == 1 && isempty(times{tt,3}) == 1 
            tm{tt,1} = [];
        else
            t1 = times{tt,1};
%             t2 = times{tt,2};
%             t3 = times{tt,3};
%             t =[t1;t2;t3]; 
            t = t1;
            tm{tt,1} = unique(t);
        end
    end
    %%%% for all the times that failed, make the cells NaN in all_data
    for iic = 1:ic
        for iiprob = 1:462
            tm_aux = tm{iiprob};
            if isempty(tm_aux) == 1
            else
                for itm = 1:length(tm_aux)
                    all_data{iiprob,tm_aux(itm),iic} = [NaN, NaN, NaN];
                end
            end
        end
    end
    
%     save(['data/failedtimes_',filename{aa},'.mat'], 'times','tm')
    save(['../data/datachop_',filename{aa},'.mat'], 'all_data','-v7.3')
    
%     clearvars -except filename aa probs
    
end
toc

%% Calculates the distance from the initial planning point (IPP) for only horizontal distances

clearvars; close all
tic

addpath("C:\Users\Larkin\planning_llh_bgc\LP model")

parameters;

for aa = 1:1 % just doing hiker

    load('Dh_km.mat')
    filename = hdist.LP;
    load(['../data/datachop_',filename{aa},'.mat'],'all_data');
    load('beh_dist_6.mat')
%     reps = 100;
    
    %%%% data format: all_data{iprob,irep,iic} = [x' y' behavior'];
    %%%% matrices for hor and ver distance from IPP for each prob and corresponding replicate
    tic
    D_h = zeros(length(probs),reps);
%     D_h2 = zeros(length(probs),reps);
%     D_h3 = zeros(length(probs),reps);
    for iprob = 1:length(probs)
        for irep = 1:reps
            x1 = all_data{iprob,irep,1}(:,1);
            y1 = all_data{iprob,irep,1}(:,2);
%             x2 = all_data{iprob,irep,2}(:,1);
%             y2 = all_data{iprob,irep,2}(:,2);
%             x3 = all_data{iprob,irep,3}(:,1);
%             y3 = all_data{iprob,irep,3}(:,2);
            if isnan(x1)
                D_h(iprob,irep) = NaN;
%                 D_h1(iprob,irep) = NaN;
%                 D_h2(iprob,irep) = NaN;
%                 D_h3(iprob,irep) = NaN;
            else
                D_h(iprob,irep) = sqrt((x1(end)-x1(3))^2+(y1(end)-y1(3))^2)*((deg2km(.015)/250)*1000);
%                 D_h1(iprob,irep) = sqrt((x1(end)-x1(3))^2+(y1(end)-y1(3))^2)*((deg2km(.015)/250)*1000);
%                 D_h2(iprob,irep) = sqrt((x2(end)-x2(3))^2+(y2(end)-y2(3))^2)*((deg2km(.015)/250)*1000);
%                 D_h3(iprob,irep) = sqrt((x3(end)-x3(3))^2+(y3(end)-y3(3))^2)*((deg2km(.015)/250)*1000);
            end
        end
    end
    
%     save(['../data/h_dist_',filename{aa},'.mat'],'D_h1','D_h2','D_h3');
    save(['../data/h_dist_',filename{aa},'.mat'],'D_h');

%     clearvars -except aa
    
end
toc

