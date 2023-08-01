% 根据地图来初始化状态列表
% 输入1：地图
% 输出1：状态列表
function state_lists = init_state_lists(map)
    % 得到地图大小
    sx = size(map, 1);
    sy = size(map, 2);
    sz = size(map, 3);
    
    % 记录每个状态值
    for xx = sx:-1:1
        for yy = sy:-1:1
            for zz = sz:-1:1
                state_lists(xx,yy,zz) = init_state(xx, yy, zz);
                state_lists(xx,yy,zz) = set_state(state_lists(xx,yy,zz), map(xx, yy, zz));
            end
        end
    end
end

