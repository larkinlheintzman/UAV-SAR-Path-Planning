function icsxy = convert_ics(iclonlat)

% Extract initial conditions
iclon = iclonlat(:,1);
iclat = iclonlat(:,2);

% load map latitude and longitude limits (from import map function)
load('mapdim.mat','LLx','LLy','latlimT','lonlimT')

% convert latitude and longitude ics to body coordinates in map
ycrds = linspace(latlimT(1),latlimT(2),LLy);
xcrds = linspace(lonlimT(1),lonlimT(2),LLx);
icsxy = zeros(size(iclonlat));
for ii = 1:size(iclonlat,1)
    iclonx = find(xcrds>=iclon(ii)-0.00005 & xcrds<=iclon(ii)+0.000005);
    iclaty = find(ycrds>=iclat(ii)-0.00001 & ycrds<=iclat(ii)+0.00005);
    icsxy(ii,:) = [iclonx iclaty];
end

save('allics.mat','iclonlat','icsxy')
save('analysis/allics_conversion.mat','iclonlat','icsxy','ycrds','xcrds')
