% 得到领域
% 输入1：当前位置 location
% 输入2：地图 maps
% 输入3：状态 states
% 输出：领域记录的值
function state_list = get_neighbor(location, map, states)
    % 得到地图大小
    sx = size(map, 1);
    sy = size(map, 2);
    sz = size(map, 3);
    
    state_list = [];

    % 遍历附近八领域
    for x = -1:1
        for y = -1:1
            for z = -1:1
                xx = location(1) + x;
                yy = location(2) + y;
                zz = location(3) + z;
                
                % 如果超出边界，拒绝采用
                if xx <= 0 || xx > sx
                    continue;
                end
                if yy <= 0 || yy > sy
                    continue;
                end
                if zz <= 0 || zz > sz
                    continue;
                end
                % 如果是本身 拒绝采用
                if x == 0 && y == 0 && z == 0
                    continue;
                end
                state_list = [state_list states(xx, yy, zz)];
            end
        end
    end
end

