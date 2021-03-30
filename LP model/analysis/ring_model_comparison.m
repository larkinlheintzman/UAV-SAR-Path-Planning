heatmap = load("C:\Users\Larkin\planning_llh_bgc\LP model\analysis\outputs\ic_1_con_hiker_t12_hmpark.csv");
heatmap = reshape(heatmap(3:end), 180, 180);
heatmap = fliplr(heatmap);

[X,Y] = meshgrid(linspace(0,1500,180), linspace(0,1500,180));

figure(1); clf; hold on;
contourf(1500 - X,Y,heatmap,7, 'linestyle','none')


colormap = parula(9);

% plt = plot(750, 750, '.', 'color', colormap(1,:), 'markersize',100);
% plt.Color(4) = 0.99;

edgeAlpha = 0.8;
faceAlpha = 0.0;

r = 80;
scatter(750,750,r*r*pi,'o','MarkerEdgeAlpha',edgeAlpha,'MarkerFaceAlpha',faceAlpha, 'markerfacecolor', colormap(1,:), 'markeredgecolor', colormap(9,:), 'linewidth',4)
r = (65/25)*r;
scatter(750,750,r*r*pi,'o','MarkerEdgeAlpha',edgeAlpha,'MarkerFaceAlpha',faceAlpha, 'markerfacecolor', colormap(4,:), 'markeredgecolor', colormap(6,:), 'linewidth',4)
r = (105/65)*r;
scatter(750,750,r*r*pi,'o','MarkerEdgeAlpha',edgeAlpha,'MarkerFaceAlpha',faceAlpha, 'markerfacecolor', colormap(6,:), 'markeredgecolor', colormap(2,:), 'linewidth',4)



plot(750,750,'p','markerfacecolor','white','markeredgecolor','black', 'markersize',32,'linewidth',2);
set(gca, 'YDir','reverse');
set(gca, 'FontSize',36);
set(gca,'TickLabelInterpreter','latex')

xlabel("cells ($6.7$m)","Interpreter","Latex");
ylabel("cells ($6.7$m)","Interpreter","Latex");

axis equal
