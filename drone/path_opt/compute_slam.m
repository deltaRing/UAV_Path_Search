% 计算碰撞系数
% 输入1：路径点Q (1 x 3)
% 输入2：障碍物顶点P (N x 3)
% 输入3：障碍物原有的顶点向量V (N x 3)
% 输出1：Jc 碰撞项惩罚
function Jc = compute_slam(Q, P, V)
    Sf   = 3.0;
    Jc   = 0.0;
    % 碰撞项惩罚初始化
    for ii = 1:size(P, 2)
        Obs = P{ii}; % 观测到的障碍物
        vObs = V{ii};
        for jj = 1:size(P{ii}, 1)
            Dij = -(Q(ii,:) - Obs(jj, :)) * vObs(jj, :)'; % 计算距离
            Cij = Sf - Dij;            % 计算Cost
            Jcij = 0;
            if Cij <= 0
                % 无损失
            elseif Cij > 0.0 && Cij <= Sf
                Jcij = Cij^3;
            elseif Cij > Sf
               Jcij = 3 * Sf * Cij^2 - 3 * Sf^2 * Cij + Sf^3;
            end
            Jc = Jc + Jcij;
            % Cost增加
        end
    end
end

