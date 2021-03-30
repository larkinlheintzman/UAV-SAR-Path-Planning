% comparing cdfs of all 462 computed pdfs vs cdfs from the LPB book
% saves the best fits for each LP type
clearvars; close all; clc;
set(0,'DefaultFigureWindowStyle','docked');
load('beh_dist_6.mat')

% %% 
% %%%%%%%%%%%%%%%% Child 1 to 3
% % choose LPT
% filename = 'child1to6'
% % define support for the pdfs
% bins = linspace(0,4000,20);
% % define exp LP type as a cdf of horizontal distance from IPP (BOOK=EXP)
% bin_width = 300;
% % what is book data?
% mobc = [0.2 0.3 0.6 4.5]*1000;                          % horizontal distance from IPP in m **************
% bin_book_max = 4500;
% num_rel_exp_pts = 5;                                    % number of points in teh exp pdf that are not close to zero
% ifig = 1;   % number of figure in this workspace
% 
% fits = LPB_bestfit(filename,bins, bin_width, mobc, bin_book_max, num_rel_exp_pts,ifig);
% bestprobs = probs(fits(1,:),:)
% %%
% %%%%%%%%%%%%%%%% Child 10 to 12
% % choose LPT
% filename = 'child10to12'
% % define support for the pdfs
% bins = linspace(0,9000,20);
% % define exp LP type as a cdf of horizontal distance from IPP (BOOK=EXP)
% bin_width = 450;
% % what is book data?
% mobc = [0.8 1.6 3.2 9.0]*1000;                          % horizontal distance from IPP in m **************
% bin_book_max = 9000;
% num_rel_exp_pts = 4;                                    % number of points in teh exp pdf that are not close to zero
% ifig = 2;   % number of figure in this workspace
% 
% fits = LPB_bestfit(filename,bins, bin_width, mobc, bin_book_max, num_rel_exp_pts,ifig);
% bestprobs = probs(fits(1,:),:)
% %%
% %%%%%%%%%%%%%%%% Dementia
% % choose LPT
% filename = 'dementia'
% % define support for the pdfs
% bins = linspace(0,8300,20);
% % define exp LP type as a cdf of horizontal distance from IPP (BOOK=EXP)
% bin_width = 415;
% % what is book data?
% mobc = [0.3 0.8 1.9 8.3]*1000;                          % horizontal distance from IPP in m **************
% bin_book_max = 8300;
% num_rel_exp_pts = 4;                                    % number of points in teh exp pdf that are not close to zero
% ifig = 3;   % number of figure in this workspace
% 
% fits = LPB_bestfit(filename,bins, bin_width, mobc, bin_book_max, num_rel_exp_pts,ifig);
% bestprobs = probs(fits(1,:),:)
%%
%%%%%%%%%%%%%%%% Hiker
% choose LPT
filename = 'hiker'
% define support for the pdfs
bins = linspace(0,18000,20);
% define exp LP type as a cdf of horizontal distance from IPP (BOOK=EXP)
bin_width = 915;%510;
% what is book data?
mobc = [1.1 3.1 5.8 18.3]*1000;                          % horizontal distance from IPP in m **************
bin_book_max = 18300;
num_rel_exp_pts = 4;                                    % number of points in teh exp pdf that are not close to zero
ifig = 4;   % number of figure in this workspace

fits = LPB_bestfit(filename,bins, bin_width, mobc, bin_book_max, num_rel_exp_pts,ifig);
bestprobs = probs(fits(1,:),:)
% %%
% % %%%%%%%%%%%%%%%% Hunter
% % choose LPT
% filename = 'hunter'
% % define support for the pdfs
% bins = linspace(0,17000,20);
% % define exp LP type as a cdf of horizontal distance from IPP (BOOK=EXP)
% bin_width = 860;
% % what is book data?
% mobc = [1 2.1 4.8 17.2]*1000;                          % horizontal distance from IPP in m **************
% bin_book_max = 17200;
% num_rel_exp_pts = 4;                                    % number of points in teh exp pdf that are not close to zero
% ifig = 5;   % number of figure in this workspace
% 
% fits = LPB_bestfit(filename,bins, bin_width, mobc, bin_book_max, num_rel_exp_pts,ifig);
% bestprobs = probs(fits(1,:),:)
% %%
% %%%%%%%%%%%%%%%% Angler
% % choose LPT
% filename = 'angler'
% % define support for the pdfs
% bins = linspace(0,9900,20);
% % define exp LP type as a cdf of horizontal distance from IPP (BOOK=EXP)
% bin_width = 495;
% % what is book data?
% mobc = [0.3 1.5 5.5 9.9]*1000;                          % horizontal distance from IPP in m **************
% bin_book_max = 9900;
% num_rel_exp_pts = 4;                                    % number of points in teh exp pdf that are not close to zero
% ifig = 6;   % number of figure in this workspace
% 
% fits = LPB_bestfit(filename,bins, bin_width, mobc, bin_book_max, num_rel_exp_pts,ifig);
% bestprobs = probs(fits(1,:),:)
% %%
% %%%%%%%%%%%%%%%% Child 13 to 15
% % choose LPT
% filename = 'child13to15'
% % define support for the pdfs
% bins = linspace(0,21400,20);
% % define exp LP type as a cdf of horizontal distance from IPP (BOOK=EXP)
% bin_width = 1070;
% % what is book data?
% mobc = [0.8 2.1 4.8 21.4]*1000;                          % horizontal distance from IPP in m **************
% bin_book_max = 21400;
% num_rel_exp_pts = 4;                                    % number of points in teh exp pdf that are not close to zero
% ifig = 7;   % number of figure in this workspace
% 
% fits = LPB_bestfit(filename,bins, bin_width, mobc, bin_book_max, num_rel_exp_pts,ifig);
% bestprobs = probs(fits(1,:),:)
% %%
% %%%%%%%%%%%%%%%% Despondent
% % choose LPT
% filename = 'despond'
% % define support for the pdfs
% bins = linspace(0,21600,20);
% % define exp LP type as a cdf of horizontal distance from IPP (BOOK=EXP)
% bin_width = 1080;
% % what is book data?
% mobc = [0.3 1.1 3.2 21.6]*1000;                          % horizontal distance from IPP in m **************
% bin_book_max = 21600;
% num_rel_exp_pts = 4;                                    % number of points in teh exp pdf that are not close to zero
% ifig = 8;   % number of figure in this workspace
% 
% fits = LPB_bestfit(filename,bins, bin_width, mobc, bin_book_max, num_rel_exp_pts,ifig);
% bestprobs = probs(fits(1,:),:)
% %%
% %%%%%%%%%%%%%%%% Snowboarder
% % choose LPT
% filename = 'snowboard'
% % define support for the pdfs
% bins = linspace(0,15400,20);
% % define exp LP type as a cdf of horizontal distance from IPP (BOOK=EXP)
% bin_width = 770;
% % what is book data?
% mobc = [1.6 3.2 6.2 15.4]*1000;                          % horizontal distance from IPP in m **************
% bin_book_max = 15400;
% num_rel_exp_pts = 5;                                    % number of points in teh exp pdf that are not close to zero
% ifig = 9;   % number of figure in this workspace
% 
% fits = LPB_bestfit(filename,bins, bin_width, mobc, bin_book_max, num_rel_exp_pts,ifig);
% bestprobs = probs(fits(1,:),:)
% %%
% %%%%%%%%%%%%%%%% Worker
% % choose LPT
% filename = 'worker'
% % define support for the pdfs
% bins = linspace(0,11800,20);
% % define exp LP type as a cdf of horizontal distance from IPP (BOOK=EXP)
% bin_width = 590;
% % what is book data?
% mobc = [1 1.9 3.2 11.8]*1000;                          % horizontal distance from IPP in m **************
% bin_book_max = 11800;
% num_rel_exp_pts = 4;                                    % number of points in teh exp pdf that are not close to zero
% ifig = 10;   % number of figure in this workspace
% 
% fits = LPB_bestfit(filename,bins, bin_width, mobc, bin_book_max, num_rel_exp_pts,ifig);
% bestprobs = probs(fits(1,:),:)