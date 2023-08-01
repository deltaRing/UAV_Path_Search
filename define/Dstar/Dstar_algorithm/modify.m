% 计算改变后的结果
% 障碍物发生变化的同时，从目标点忘起始点回推，更新由于障碍物发生变化而引起的路径代价变化
% 输入1：Dstar D*算法主要class
% 输入2：state 状态
% 输出1：Dstar
function Dstar = modify(Dstar, state)
    Dstar = modify_cost(Dstar, state);
    while 1
        [kmin, Dstar] = process_state(Dstar);
        if kmin >= state.h
            break
        end
    end
end
