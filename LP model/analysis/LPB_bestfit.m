function fits = LPB_bestfit(filename,bins, bin_width, mobc, bin_book_max, num_rel_exp_pts,ifig)

addpath("C:\Users\Larkin\planning_llh_bgc\LP model")
parameters;
% global probs N reps
% load behavior distributions for bar plots
load('beh_dist_6.mat','probs')
% number of LPB types
% N = 462;
% number of reps
% reps = 100;
% number of initial conditions
% ics = 3;
size(ics,1) = 1;

isrid_bins = [.25 .5 .75 .95];                          % percentiles
mob_bins = 0:bin_width:bin_book_max;                    % define bins to compute cdfs
mobc = interp1([0,mobc],[0,isrid_bins],mob_bins );      % interpolate cdf at bins
mobp = diff(mobc)/sum(diff(mobc));                      % differentiate cdf to get pdf
mob_binsP = bin_width/2:bin_width:bin_book_max;         % define bins for pdf (centers from cdf bins)

% define bin centers to use for pdfs and cdfs
bin_centers = bins(1:end-1)+bins(2)-bins(1);
% compute range to compare pdfs
M1 = max(min(bin_centers),min(mob_binsP));               % find minimum of overlap range between exp and model cdfs
M2 = min(max(bin_centers),max(mob_binsP));               % find maximum of overlap range between exp and model cdfs

% define markers for plot
markers={'p','v','s','o','^','d'};

% load distance data
% load(['../data/h_dist_',filename,'.mat'],'D_h1','D_h2','D_h3'),
% D_h = {D_h1, D_h2, D_h3};
load(['../data/h_dist_',filename,'.mat'],'D_h'),
D_h = {D_h};

%%%%%%%%%%%%%%%%%%%%% ICs %%%%%%%%%%%%%%%%%%%%%%%%%%%%
for jj = 1:size(ics,1)
    
    % compare model pdfs to LPB types
    for ii = 1:N
        if isnan(D_h{jj}(ii,:))
            D_aux = rmmissing(D_h{jj}(ii,:));
            auxreps =length(D_aux);
            [mod,edge] = histcounts(D_aux,mob_bins);        % mod is the model simulation pdf, normalized
            mod = mod/auxreps;                              % normalize simulation data
            exp = mobp;                                     % set experimental pdf to one found above
            
            ind = exp.*mod==0;                              % find where model and exp pdfs are zero (and KL divergence breaks down)
            exp(ind)=[];                                    % omit those entries from exp
            mod(ind)=[];
            
            dist(ii) = KLDiv(mod,exp) + KLDiv(exp,mod);     % compute KL div to see "distance" between pdfs
            distN(ii) = length(exp);                        % used to make sure model pdfs are same length as exp pdf
        else
            [mod,edge] = histcounts(D_h{jj}(ii,:),mob_bins);        % mod is the model simulation pdf, normalized
            mod = mod/reps;                                         % normalize simulation data
            exp = mobp;                                             % set experimental pdf to one found above
            
            ind = exp.*mod==0;                              % find where model and exp pdfs are zero (and KL divergence breaks down)
            exp(ind)=[];                                    % omit those entries from exp
            mod(ind)=[];
            
            dist(ii) = KLDiv(mod,exp) + KLDiv(exp,mod);     % compute KL div to see "distance" between pdfs
            distN(ii) = length(exp);                        % used to make sure model pdfs are same length as exp pdf
        end
    end
    
    % which LPB type matches best?
    [KLmin,IDmin]=sort(dist);       % KLmin is the distances between pdfs and IDmin is the indices for probs
    %         IDmin(1:5)            % Probs with the lowest KL div distance
    y = IDmin(KLmin~=0);            % y will make sure the best fit pdf is not a dist equal to zero
    n = find(distN(y)>=num_rel_exp_pts);     % to check that the exp and model pdfs have at least 4 points
    %             y(1:5)                     % Probs with the lowest KL div distance not zero
    
    % check if the best fit is 84 (all DT)
    flag84 = zeros(1,3);
    auxbf = y(n);
    bf = [];
    for icheck = 1:3                %%%% if the bf is 84, flag it, and choose next best
        if auxbf(icheck) == 84
            flag84(icheck) = 1;
            bf(icheck) = auxbf(icheck+1);
            auxbf(icheck) =[];
        else
            bf(icheck) = auxbf(icheck);
        end
    end
    
    % plot best fitting pdf: the best fit pdf is the lowest KL div distance
    % that's not zero and has the same amount of points as exp pdf
    dist(bf)
    ii = bf(1);                                         % best fitting pdf probability
    [mod,edge] = histcounts(D_h{jj}(ii,:),mob_bins);    % finding the model pdf for the best fit
    mod = mod/reps;                                     % normalize simulation data
    exp = mobp;                                         % exp pdf doesn't change but needs to reset since it was shortened for KL div calculation
    x = linspace(M1,M2,20);
    %     exp = interp1(mob_binsN,mobp,x);              % interpolate experimental pdf on overlap *********************************
    %     mod = interp1(bin_centers,pdf(ii,:),x);       % interpolate model pdf on overlap
    ind = exp.*mod==0;                                  % find where model and exp pdfs are zero (and KL divergence breaks down)
    exp(ind)=[];                                        % omit those entries from exp
    mod(ind)=[];                                        % omit those entries from model
    
    fits(:,jj) = bf';     % each column is an IC, row 1 is the best fit for each IC
    bfKL(:,jj) = dist(bf);
    flagallDT(:,jj) = flag84';
    
    %% Figures
    %%%%% figure with database PMF and 5 model PMFs for four ICs
    %     cc = parula(5);
    %     figure(ifig),
    %     if jj == 1
    %         plot(x(1:length(exp)),exp,strcat('-',markers{jj}),'color',cc(jj,:)), hold on, grid on
    %         plot(x(1:length(exp)),mod,strcat('--',markers{jj+1}),'color',cc(jj,:))
    %     else
    %         plot(x(1:length(exp)),mod,strcat('--',markers{jj+1}),'color',cc(jj,:))
    %     end
    
    
    %%%%% figure of bar plots sorted by the best fit probabilty dist
    %             figure(ifig+11),
    %             if jj==1
    %                 subplot(size(ics,1),1,jj),
    %                         bar(probs(y(n),:),'stacked','edgecolor','k', 'barwidth',1)
    %                 axP = get(gca,'Position');
    %                 legend('Random Traveling ','Route Traveling ','Direction Traveling ','Staying Put ','View Enhancing ','Backtracking','orientation','horizontal','location','northoutside')
    %                 set(gca, 'Position', axP+[0 0 .05 0])
    %                 ylabel('IC 2'), xlabel([])
    %             else
    %                 subplot(size(ics,1),1,jj),
    %                 bar(probs(y(n),:),'stacked','edgecolor','k', 'barwidth',1)
    %                 axP = get(gca,'Position');
    %                 set(gca, 'Position', axP+[0 0 .05 0])
    %                 ylabel(['IC ',num2str(jj+1)]),xlabel([])
    %                 if jj == size(ics,1)
    %                     xlabel('Behavior distributions ranked by KL divergence')
    %                 end
    %             end
    %             suptitle(['LPT ',filename])
    %             set(gca,'fontname','times','fontsize',14)
    
    %%%%% figure of the best fits probs sorted from smallest to largest
    %             figure(ifig+22),
    %             ax = 1:length(KLmin);
    %             pt = find(KLmin~=0);
    %             pt = pt(n);
    %             plot(ax,KLmin,'-o'),
    %             hold on
    %             plot(pt(1:3),bfKL,'rp','markersize',10)
    %             xlim([0 462]), xlabel('best fits probs sorted from smallest to largest')
    %             ylabel('KL divergence')
    %             title(['LPT ',filename])
    %             set(gcf,'PaperPosition',[0,0,11,8],'paperorientation','landscape');
    %
    %             print('-dpdf',['plots/',filename,'_ic',num2str(jj),'_kldiv.pdf'])
end

%% Printing figures
% figure(ifig),
% xlabel('Horizontal distance (meters)'), ylabel('PMF'), title(['LPT ',filename])
% legend('From ISRID',['Model with IC 1, fit ',num2str(fits(1,1))],...
%     ['Model with IC 2, fit ',num2str(fits(1,2))],...
%     ['Model with IC 3, fit ',num2str(fits(1,3))]),...
%     ['Model with IC 4, fit ',num2str(fits(1,4))])
% legend('From ISRID',['Model with IC 1, fit ',num2str(fits(1,1))])
% legend('From ISRID',['Model with IC 2, fit ',num2str(fits(1,1))],...
%     ['Model with IC 3, fit ',num2str(fits(1,2))])%,...
%     ['Model with IC 4, fit ',num2str(fits(1,4))])

% set(gca,'fontname','times','fontsize',14)
% set(gcf,'PaperPosition',[0,0,6,5],'paperorientation','portrait');
% print('-painters','-dmeta',[filename,'best_pmfs.emf'])
% print('-painters','-dpng',['all size(ics,1)/plots/',filename,'_pmfs.png'])
% print('-painters','-dpdf',['plots/',filename,'_pmfs.pdf'])

% figure(ifig+11),
% set(gca,'fontname','times','fontsize',14)
% set(gcf,'PaperPosition',[0,0,11,8],'paperorientation','landscape');
% print('-painters','-dmeta',[filename,'sorted_barplots.emf'])
% set(gcf,'PaperPosition',[0,0,11,9],'paperorientation','landscape');
% print('-painters','-dpng',['plots/',filename,'sorted_barplots.png'])


save(['../data/fits_',filename,'.mat'],'fits','bfKL','probs','flagallDT')
end
