% �ҵ���Сk��min_state�ڵ�
% ����1��Dstar D*�㷨��Ҫclass
% ���1��minState ��С��״̬
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