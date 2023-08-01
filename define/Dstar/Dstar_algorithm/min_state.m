% 找到最小k的min_state节点
% 输入1：Dstar D*算法主要class
% 输出1：minState 最小的状态
function minState = min_state(Dstar)
    if isempty(Dstar.openList), minState = []; return; end
    kmin_ = inf;
    minState = [];
    for ii = 1:length(Dstar.openList)
        if kmin_ > Dstar.openList(ii).k, 
            kmin_     = Dstar.openList(ii).k;
            minState = Dstar.openList(ii);
        end
    end
end