% 计算光滑项目
% 输入：Q [N x 3]
% 输出：光滑项代价 Js
function Js = compute_smooth(Q)
    path_size = size(Q, 1);
    
    dt = 0.333; % 最快3m/s
    V = []; A = []; J = [];
    Js = 0.0;
    if path_size < 3
        return;
    end
    % 求解速度
    for qq = 1:path_size - 1
        V = [V; (Q(qq + 1, :) - Q(qq, :)) / dt];
    end
    % 根据速度求得加速度
    for vv = 1:path_size - 2
        A  = [A; (V(vv + 1, :) - V(vv, :)) / dt];
        Js = Js + norm((V(vv + 1, :) - V(vv, :)) / dt);
    end
    % 根据加速度求得加加速度
    for aa = 1:path_size - 3
        J  = [J; (A(aa + 1, :) - A(aa, :)) / dt]; 
        Js = Js + norm((A(aa + 1, :) - A(aa, :)) / dt);
    end
end

