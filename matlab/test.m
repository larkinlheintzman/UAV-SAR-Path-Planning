
axis = 0:0.1:20;


f = zeros(size(axis, 2));

for y = 0:0.1:20
    for x = 0:0.1:20
        f(y, x) = exp( -(x.^2) + (y.^2) );
    end
end
% % h = 0:0.05:10;
% for h = 0:0.5:10
% %     y = exp(-x.^2);
%     yh = exp(-x.^2./(h+1).^(2));
% 
%     plot(x, y);
%     hold on;
% 
%     plot(x, yh);
% end
% 
% % z = exp(-repmat(x, 101, 1).^2 ./ repmat(h', 1, 201).^2 );