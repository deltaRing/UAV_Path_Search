% 计算障碍物到路径点的向量
% 输入1：地图Map
% 输入2：原有路径Path (should be N x 3)
% 输入3：新探测到的障碍物索引 (should be N x 3)
% 输出1：P 建筑物的控制点   (应该是一个)   M x (? x 3)
% 输出2：V 控制向量         (应该是一个)   M x (? x 3)
% 输出3：Q 路径
% 输出4：距离
function [P, V, Q, D] = GetObstacleVector(Map, Path, NewObstacle)
    map_define;
    P = []; V = []; Q = []; D = [];
    % 如果路径点大小小于3 无结果返回
    if size(Path, 1) <= 3, return; end
    % 当前的数据索引
    index = 1;
    % 加载地图定义
    for pp = 1:size(Path, 1)
        px = Path(pp, 1); py = Path(pp, 2); pz = Path(pp, 3);
        if Map(px, py, pz) ~= obstacle
            continue
            % 需要寻找到障碍物的节点
        elseif Map(px, py, pz) == destination
            break;
        end
        % 路径得到的是障碍物了
        Q_t = []; V_t = []; P_t = []; D_t = [];
        
        for ppp = 1:size(NewObstacle, 1)
            ppx = NewObstacle(ppp, 1); 
            ppy = NewObstacle(ppp, 2);
            ppz = NewObstacle(ppp, 3);
            
            Vpp = [ppx - px ppy - py ppz - pz]; 
            Vpp = Vpp / norm(Vpp);
            dpp = [ppx - px ppy - py ppz - pz] * Vpp';
            if dpp < 0
                continue;
            end
            Q_t = [Q_t; px py pz];
            P_t = [P_t; ppx ppy ppz];
            V_t = [V_t; Vpp(1) Vpp(2) Vpp(3)];
            D_t = [D_t; dpp];
        end
        P{index} = P_t;
        V{index} = V_t;
        Q{index} = Q_t;
        D{index} = D_t;
        index = index + 1;
    end
    
end

