% 绘制非未知区域的散点
% 输入1：observed 观测结果
function plot_observed(observed)
    map_define;
    % 找到合理的位置
    sx = size(observed, 1);
    sy = size(observed, 2);
    sz = size(observed, 3);
    
    % 绘制观测点
    xx = []; yy = []; zz = [];
    for ii = 1:sx
        for jj = 1:sy
            for kk = 1:sz
                if observed(ii, jj, kk) == obstacle
                    xx = [xx ii];
                    yy = [yy jj];
                    zz = [zz kk];
                end
            end
        end
    end

    scatter3(xx, yy, zz, 0.5)
    axis([0 sx 0 sy 0 sz])
    hold on
end

