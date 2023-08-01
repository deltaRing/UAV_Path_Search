% 删除对应的state
% 输入1：Dstar D*算法主要class
% 输入2：state 状态
% 输出1：Dstar
function Dstar = remove_state(Dstar, state)
    map_define;
    % 加载地图定义
    if state.t == open_
        state.t = close_;
    end
    
    for ii = 1:length(Dstar.openList)
        s = Dstar.openList(ii);
        if state.x == s.x && state.y == s.y && state.z == s.z
            Dstar.openList(ii) = [];
            Dstar.states(state.x, state.y, state.z) = state;
            break;
        end
    end
end