% 添加新状态到openList里面
% 输入1：Dstar D*算法主要class
% 输入2：state 状态
% 输入3：h_new H值
% 输出1：Dstar
function Dstar = insert_state(Dstar, state, h_new)
    % 运行地图初始化
    map_define;
    if state.t == new
        state.k = h_new;
    elseif state.t == open_
        state.k = min([state.k, h_new]);
    elseif state.t == close_
        state.k = min([state.h, h_new]);
    end
    
    state.h = h_new;
    state.t = open_;
    sX = state.x; sY = state.y; sZ = state.z;
    
    % 将本列表中所有的数值进行遍历 确定没有重复的列表
    for ii = 1:length(Dstar.openList)
        s = Dstar.openList(ii);
        if s.x == sX && s.y == sY && s.z == sZ
            % 更新state
            Dstar.openList(ii) = state;
            Dstar.states(sX, sY, sZ) = state;
            return;
        end
    end
    
    Dstar.states(sX, sY, sZ) = state;
    Dstar.openList = [Dstar.openList state];
end
