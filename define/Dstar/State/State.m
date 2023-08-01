classdef State
    %STATE 定义的D star 状态
    
    % 数值变量
    properties
        state = [];  % 地图状态
        parent = []; % 上个节点
        x = -1;      % X位置
        y = -1;      % Y位置
        z = -1;      % Z位置
        t = 0;       % 这是 close 状态 还是 open 状态 还是 new 状态
        h = 0;
        k = 0;
    end
    
    methods
        function obs = State(xx, yy, zz)
            map_define;
            % 加载定义
            obs.x = xx;
            obs.y = yy;
            obs.z = zz;
            obs.parent = [];
            obs.state = empty;
            obs.t = new;
            obs.h = 0;
            obs.k = 0;
        end
        
        % 得到距离值 该距离值是由欧氏距离直接决定的
        function value = cost(obj, state)
            map_define;
            % 加载定义
            if obj.state == obstacle, value = MAX_VALUE; return; end
            value = norm([obj.x - state.x, obj.y - state.y, obj.z - state.z]);
        end
        
        % 设置该单元的状态
        function obj = set_state(obj, state)
            obj.state = state;
        end
    end
end
