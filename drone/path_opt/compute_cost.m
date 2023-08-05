% 计算所有来源代价的
% 代价来源1：碰撞可能性代价 
% 代价来源2：动力学代价（速度、加速度、加加速度）
% 代价来源3：可行性代价
% 输入1： 路径          Q (M x 3)
% 输入2： Q遇到的障碍物 P  (M x N x 3)
% 输入3： Q遇到的障碍物所对应的向量 V (M x N x 3)
% 输出1： 所有的代价    (float Number)
function cost = compute_cost(Q, P, V)
    cost = 2.0 * compute_dynamic(Q) + 50.0 * compute_slam(Q,P,V) + 5.0 * compute_smooth(Q);
end

