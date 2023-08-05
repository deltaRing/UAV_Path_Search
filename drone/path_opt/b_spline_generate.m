% 根据Dstar的曲线 生成B样条曲线
% 输入1： Path 根据DStar算法预先得到的路径
% 输入2： dt
% 输入3： order 几阶的B样条曲线
% 输出4： new_path 最新的b样条路径
function new_path = b_spline_generate(Path_point, dt, order)
    if nargin == 1
        dt = 0.01;
        order = 3;
    end
    % 最新的B样条路径赋予初值
    new_path = []; 
    n = size(Path_point, 1) - 1;
    % data 数据数目
    
    if n < order
        disp('Data size must be greater than order')
        return;
    end
    
    % 遍历每个路径点
    NodeVector = linspace(0, 1, n + order + 1); % 节点矢量
    % t 属于 0 ---> 1
    for t = (order-1)/(n+order+1):dt:(n+2) / (n+order+1)
        % 预先定义数据
        pts = [];
        Bs  = [];
        % 计算样条
        for tt = 0:1:n % 舍弃最后一个数据
            pt  = Path_point(tt + 1, :);
            B   = N(t, NodeVector, tt, order - 1);
            pts = [pts; pt];
            Bs  = [Bs B];
        end
        xxx = Bs * pts(:,1); yyy = Bs * pts(:,2); zzz = Bs * pts(:,3);
        new_path = [new_path; xxx yyy zzz];
    end 
end

% 计算基函数
% 输入1：t 当前的t
% 输入2：k 点数
% 输入3：m 阶数
% 输出1：weight 权重
function weight = N(t, NodeVector, k, m)
    if m == 0
        if t >= NodeVector(k+1) && t <= NodeVector(k+2)
            weight = 1; 
            return;
        else
            weight = 0;
            return;
        end
    else
        L1 = (NodeVector(k + m + 1) - NodeVector(k + 1));
        L2 = (NodeVector(k + m + 2) - NodeVector(k + 2));
        if L1 == 0, L1 = 1; end % 规定了 0 / 0 = 0
        if L2 == 0, L2 = 1; end
        Bk_M_1   = N(t, NodeVector, k, m - 1);
        Bk_1_M_1 = N(t, NodeVector, k + 1, m - 1);
        weight = (t - NodeVector(k + 1)) / L1 * Bk_M_1 + ...
            (NodeVector(m + k + 2) - t) / L2 * Bk_1_M_1;
        return;
    end
end