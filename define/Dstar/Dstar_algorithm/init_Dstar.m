% Dstar 算法初始化
% 输入1：map 当前场景的地图
% 输入2：state 当前场景的状态
% 输出：当前场景的 Dstar 算法结构体
function Dstar = init_Dstar(map, states)
    Dstar.map = map;
    Dstar.states = states;
    Dstar.openList = [];
end

