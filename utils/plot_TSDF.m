% 本函数允许我观测TSDF的变化
% 以及TSDF_weight的变化
% 输入1：TSDF OR TSDF_weight
% 输入2：需要观测的值（向量）
function plot_TSDF(map, value)
    sx = size(map, 1);
    sy = size(map, 2);
    sz = size(map, 3);
    [xx, yy, zz] = meshgrid([1:sx]);

    for ii = 1:length(value)
        % 取出对应的值
        v = value(ii);
        isosurface(xx, yy, zz, map, v);
        hold on
    end
    axis([0 sx 0 sy 0 sz])
end

