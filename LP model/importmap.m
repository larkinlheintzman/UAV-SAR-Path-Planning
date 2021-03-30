function [sZelev, BWLF, BWInac] = importmap(fnameelev,fnamew,fnamer,fnamel)
%%%% Function - Import map data and all layers for linear features and inaccessible
% Layers: 1. Watershed - rivers that are crossable
%         2. Lake - rivers and lakes that have linear feature edges but
%            are inaccessible within the borders
%         3. Roads - primary and secondary linear features

latlonlim = str2double(strsplit(fnamew,{'/',',','_'},'CollapseDelimiters',true));
latlimT = [latlonlim(5) latlonlim(3)];
lonlimT = [latlonlim(2) latlonlim(4)];

%% Watershed
val = jsondecode(fileread(fnamew));
allpaths = val.paths;

% Put all the path lat-lon coords into a length x 2 matrix
paths = cell2mat(val.paths);

% Read in table of elevation data sampled 200x200
T = readtable(fnameelev);
A = table2array(T);
long = reshape(A(:,1),[200,200]);
lati = reshape(A(:,2),[200,200]);
elev = reshape(A(:,3),[200,200]);

% INTERPOLATE elevation - create a 200x200 grid in these limits where each cell is 6.7m
cellslat = deg2km(latlimT(2)-latlimT(1))/0.0067;
cellslon = deg2km(lonlimT(2)-lonlimT(1))/0.0067;
[Xq,Yq] = ndgrid(0:200/cellslat:200,0:200/cellslon:200);
[X,Y] = ndgrid(1:200,1:200);
Felev = griddedInterpolant(X,Y,elev);
Flon = griddedInterpolant(X,Y,long);
Flat = griddedInterpolant(X,Y,lati);
Zelev = Felev(Xq,Yq);
Zlon = Flon(Xq,Yq);
Zlat = Flat(Xq,Yq);

% reorganize the water data so that the paths are preserved so I have to
% take out all of the points outside of the elevation area first
lonpts = Zlon(1,:);
latpts = Zlat(:,1);
auxpaths = allpaths;
for ii=1:length(allpaths)
    lonin = allpaths{ii}(:,1) >= min(lonpts) & allpaths{ii}(:,1) <= max(lonpts);      % time indices where x is in the box
    for jj = 1:length(lonin)
        latin = allpaths{ii}(jj,2) >= min(latpts) & allpaths{ii}(jj,2) <= max(latpts);
        if lonin(jj) == 1 && latin == 0
            auxpaths{ii}(jj,:) = NaN;
        elseif lonin(jj) == 0
            auxpaths{ii}(jj,:) = NaN;
        end
        
    end
end
auxm = auxpaths;
for ii = 1:length(auxm)
    lencell = length(auxm{ii});
    for jj = 1:lencell
        if any(isnan(auxm{ii}(lencell-jj+1,:)))
            auxm{ii}(lencell-jj+1,:) = [];
        end
    end
end
index = cellfun(@isempty, auxm) == 0;
finalpaths = auxm(index);

% check with figure
figure,
for ii = 1:length(finalpaths)
    plot(finalpaths{ii}(:,1),finalpaths{ii,1}(:,2),'k')
    hold on
end

%%%%% Take the figure of the paths lines and binarize it to find logical
%%%%% matrix of 1s and 0s to "add" to the linear feature BW matrix.
xlim(lonlimT)
ylim(latlimT)
daspect([1 1 1])
set(gca,'linewidth',1)
axis off
t = gcf;
exportgraphics(t,'water.png')
I = imread('water.png');
Igray = rgb2gray(I);
sI = size(Igray);

%%%%% Take the original Igray image & set 255 = 0 everywhere and 1 everywhere else
ig = double(Igray);
ind1 = find(ig==255);
ind2 = find(ig~=255);
ig(ind1) = 0;
ig(ind2) = 1;
% ig = ig';
sI = size(ig);

%%%%% Interpolate the water path ig to the same size as Zelev and combine
[Xqi,Yqi] = ndgrid(0:(sI(1))/cellslat:sI(1),0:(sI(2))/cellslon:sI(2));
[Xi,Yi] = ndgrid(1:sI(1),1:sI(2));
Fi = griddedInterpolant(Xi,Yi,ig);
Zi = Fi(Xqi,Yqi);
BWwater = bwmorph(Zi,'thin',Inf);           %%%%%%% SETS LINES TO 1 CELL WIDTH

%%%%% SMOOTH gradient of elevation
sZ = size(Zelev);
Zf = flipud(Zelev);
sigma = 2;
sZelev = imgaussfilt(Zf,sigma);
sZgrad = imgradient(sZelev,'CentralDifference');
BWs = edge(sZgrad,'canny',[0.01 0.3]);
BWelevationGrad = double(BWs);

%% Roads
valr = jsondecode(fileread(fnamer));
allroads = valr.paths;

% Put all the path lat-lon coords into a length x 2 matrix
roads = cell2mat(allroads);

% take out all of the points outside of the elevation area first
auxroads = allroads;
for ii=1:length(allroads)
    lonin = allroads{ii}(:,1) >= min(lonpts) & allroads{ii}(:,1) <= max(lonpts);      % time indices where x is in the box
    for jj = 1:length(lonin)
        latin = allroads{ii}(jj,2) >= min(latpts) & allroads{ii}(jj,2) <= max(latpts);
        if lonin(jj) == 1 && latin == 0
            auxroads{ii}(jj,:) = NaN;
        elseif lonin(jj) == 0
            auxroads{ii}(jj,:) = NaN;
        end
        
    end
end
auxr = auxroads;
for ii = 1:length(auxr)
    lencell = length(auxr{ii});
    for jj = 1:lencell
        if any(isnan(auxr{ii}(lencell-jj+1,:)))
            auxr{ii}(lencell-jj+1,:) = [];
        end
    end
end
index = cellfun(@isempty, auxr) == 0;
finalroads = auxr(index);

% check with figure
figure,
for ii = 1:length(finalroads)
    plot(finalroads{ii}(:,1),finalroads{ii,1}(:,2),'k')
    hold on
end

%%%%% Take the figure of the paths lines and binarize it to find logical
%%%%% matrix of 1s and 0s to "add" to the linear feature BW matrix.
xlim(lonlimT)
ylim(latlimT)
daspect([1 1 1])
set(gca,'linewidth',1)
axis off
tr = gcf;
exportgraphics(tr,'roads.png')
Ir = imread('roads.png');
Igrayr = rgb2gray(Ir);

%%%%% Take the original Igray image & set 255 = 0 everywhere and 1 everywhere else
igr = double(Igrayr);
ind1 = find(igr==255);
ind2 = find(igr~=255);
igr(ind1) = 0;
igr(ind2) = 1;
sIr = size(igr);

%%%%% Interpolate the road path ig to the same size as Zelev and combine
[Xqir,Yqir] = ndgrid(0:(sIr(1))/cellslat:sIr(1),0:(sIr(2))/cellslon:sIr(2));
[Xir,Yir] = ndgrid(1:sIr(1),1:sIr(2));
Fir = griddedInterpolant(Xir,Yir,igr);
Zir = Fir(Xqir,Yqir);
BWroads = bwmorph(Zir,'thin',Inf);           %%%%%%% SETS LINES TO 1 CELL WIDTH

%% Lakes
val1 = jsondecode(fileread(fnamel));
allrings = val1.rings;

% Put all the path lat-lon coords into a length x 2 matrix
poly = cell2mat(allrings);

% reorganize the lake data so that the paths are preserved so I have to
% take out all of the points outside of the elevation area first
lonpts = Zlon(1,:);
latpts = Zlat(:,1);
auxrings = allrings;
for ii=1:length(allrings)
    lonin = allrings{ii}(:,1) >= min(lonpts) & allrings{ii}(:,1) <= max(lonpts);      % time indices where x is in the box
    for jj = 1:length(lonin)
        latin = allrings{ii}(jj,2) >= min(latpts) & allrings{ii}(jj,2) <= max(latpts);
        if lonin(jj) == 1 && latin == 0
            auxrings{ii}(jj,:) = NaN;
        elseif lonin(jj) == 0
            auxrings{ii}(jj,:) = NaN;
        end
        
    end
end
auxm1 = auxrings;
for ii = 1:length(auxm1)
    lencell = length(auxm1{ii});
    for jj = 1:lencell
        if any(isnan(auxm1{ii}(lencell-jj+1,:)))
            auxm1{ii}(lencell-jj+1,:) = [];
        end
    end
end
index1 = cellfun(@isempty, auxm1) == 0;
finalrings = auxm1(index1);

%%%%% Check that the first and last points of the chopped ring data are the same
finalrings1 = finalrings;
for ii = 1:length(finalrings1)
    if ~isequal(finalrings1{ii}(1,:),finalrings1{ii}(end,:))
        finalrings1{ii}(end+1,:) = finalrings{ii}(1,:);
    end
end

%%%%% Interpolate the lake data cell by cell to preserve the rings
% Find the boundaries of each of the rings - these are the LFs
% we should add all these together in one matrix for the LFs lakes
close all; 
allbounds = cell(length(finalrings1),1);
outbounds = cell(length(finalrings1),1);
for ilake = 1:length(finalrings1)
    figure,
    plot(finalrings1{ilake}(:,1),finalrings1{ilake,1}(:,2),'k')
    
    %%%%% Take the figure of the paths lines and binarize it
    xlim(lonlimT), ylim([latlimT(1)-0.001,latlimT(2)]), daspect([1 1 1])
    set(gca,'XTick',[], 'YTick', [])
    exportgraphics(gcf,'lake.png','ContentType','image','resolution',300)
    Il = imread('lake.png');
    Igrayl = rgb2gray(Il);
    
    % crop the axis border from the image %%%%%% this is a temporary fix
    Igrayl(end-4:end,:) = [];
    Igrayl(:,end-4:end) = [];
    Igrayl(1:5,:) = [];
    Igrayl(:,1:5) = [];
    sIl = size(Igrayl);
    
    %%%%% Take the original Igray image & set 255 = 0 everywhere and 1 everywhere else
    igl = double(Igrayl);
    ind1 = find(igl==255);
    ind2 = find(igl~=255);
    igl(ind1) = 0;
    igl(ind2) = 1;
    BWlake = igl;
    
    %%%%% Interpolate the lake path igl to the same size as Zelev
    [Xqil,Yqil] = ndgrid(0:(sIl(1))/cellslat:sIl(1),0:(sIl(2))/cellslon:sIl(2));
    [Xil,Yil] = ndgrid(1:sIl(1),1:sIl(2));
    Fil = griddedInterpolant(Xil,Yil,BWlake);
    Zil = Fil(Xqil,Yqil);
    sZil = size(Zil);

    %%%%%%%%%%%%%%%%%%%%%%%%%%% Boundaries %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % BWbound contains the coordinates for the boundary as a LF.
    % B is a cell array of k objects and holes with B{1,1} as the outer
    % boundary, B{2,1} and B{3,1} as the boundaries of the holes. L is a 2D
    % matrix with 1's as the riverbanks (multiple cells wide) and 2's and 3's as
    % the holes, and these include the boundaries. These are filled in areas.
    % Only the outer boundary B{1,1} will be the LF. All inside 1's 2's and
    % 3's will be inaccessible. N are number of objects.
    [B,L,N] = bwboundaries(Zil,'holes');
    
    % just the outer boundaries -- Linear Features
    BWbound = zeros(sZ);
    for jj = 1:N
        indB = B{jj};
        for tt = 1:length(indB)
            BWbound(indB(tt,1),indB(tt,2)) = 1;
        end
    end
    % inside the bounds and also the holes -- inaccessible
    L(BWbound == 1) = 0;
    L(L ~= 0) = 1;
    BWinaccessible = L;
    
    allbounds{ilake} = BWbound;
    outbounds{ilake} = BWinaccessible;
    
end
%%%% Sum together all the boundaries and inaccessible lake areas
n = size(allbounds,1);
% Get size of matrices in cell.
matSize = size(allbounds{1});
auxall = reshape(cell2mat(allbounds'),matSize(1),matSize(2),n);
% Sum 3D matrix along 3rd dimension
allboundssum = sum(auxall,3);
allboundssum(allboundssum~=0) = 1;

auxout = reshape(cell2mat(outbounds'),matSize(1),matSize(2),n);
outboundssum = sum(auxout,3);
outboundssum(outboundssum~=0) = 1;

BWlakeLF = allboundssum;
BWlakeInac = outboundssum; 

%% saving all the BW matrices and edges for plots
BWLF = BWelevationGrad + BWwater + BWlakeLF + BWroads;
BWInac = BWlakeInac;
BWLF(BWLF ~= 0) = 1;
BWInac(BWInac ~= 0) = 1;

save('allBW.mat','BWelevationGrad','BWwater','BWlakeLF','BWlakeInac','BWroads','BWLF','BWInac')
save('BW_LFandInac_Zelev.mat','BWLF','BWInac','sZelev')

% map dimensions
LLx = size(BWLF,2);
LLy = size(BWLF,1);
save('mapdim.mat', 'LLx','LLy','latlimT','lonlimT')

%% Plots

% % Canny Edge detection 
% allBW = {BWelevationGrad; BWwater; BWlakeLF; BWlakeInac; BWroads; BWLF; BWInac};
% allEdges = cell(length(allBW),1);
% for ij = 1:length(allBW)
%     BW = allBW{ij};
%     E=[];
%     for ii=1:sZ(1)
%         for jj=1:sZ(2)
%             if BW(ii,jj) ==1
%                 E = [E; [ii,jj]];
%             else
%             end
%         end
%     end
%     allEdges{ij} = E;
% end
% Eelevationgrad = allEdges{1};
% Ewater = allEdges{2};
% ElakeLF = allEdges{3};
% ElakeInac = allEdges{4};
% Eroads = allEdges{5};
% % EelevationInac = allEdges{6};
% EallLF = allEdges{6};
% EallInac = allEdges{7};
% 
% save('edges_all.mat','Eelevationgrad','Ewater','ElakeLF','ElakeInac','Eroads','EallLF','EallInac')

% %% Plots of all the edges
% load('edges_all.mat')
% allE = {Eelevationgrad;Ewater;ElakeLF;ElakeInac; Eroads};
% 
% figure,
% ax = axes();
% E = allE{1};
% h(1) = plot(E(:,2),E(:,1),'k.','markersize',1); daspect([1 1 1])
% hold on
% E = allE{2};
% h(2) = plot(E(:,2),E(:,1),'.','markersize',1);
% E = allE{3};
% h(3) = plot(E(:,2),E(:,1),'.','markersize',1);
% E = allE{4};
% h(4) = scatter(E(:,2),E(:,1),'filled','MarkerFaceColor','b'); 
% alpha(h(4),0.009)
% E = allE{5};
% h(5) = plot(E(:,2),E(:,1),'.','markersize',1);
% title('Linear Features and Inaccessible Areas')
% legend('elevation gradient LFs','water LFs','riverbank LFs','inaccessible water','road LFs')
% set(gca,'fontsize',12)
% % copy the objects
% hCopy = copyobj(h, ax); 
% % replace coordinates with NaN
% set(hCopy,'XData', NaN', 'YData', NaN)
% % Alter the graphics properties
% hCopy(1).MarkerSize = 15; 
% hCopy(2).MarkerSize = 15; 
% hCopy(3).MarkerSize = 15; 
% hCopy(4).LineWidth = 15; 
% hCopy(5).MarkerSize = 15; 


end