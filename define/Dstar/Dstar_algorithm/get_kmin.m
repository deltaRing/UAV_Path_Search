
% 在OpenList里面最小的k
% 输入1：Dstar D*算法主要class
% 输出1：kmin 
function kmin = get_kmin(Dstar)
    if isempty(Dstar.openList), kmin = -1; return; end
    kmin = inf;
    for ii = 1:length(Dstar.openList)
        if kmin > Dstar.openList(ii).k
            kmin = Dstar.openList(ii).k;
        end
    end
end
