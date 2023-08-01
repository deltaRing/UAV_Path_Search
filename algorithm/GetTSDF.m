% 获得TSDF信息
% 输入1：当前地图
% 输入2：当前TSDF地图
% 输入3：当前权重
% 输入4：当前位置
% 输入5：当前方位角
% 输入6：当前俯仰角
% 输入7：距离最小值
% 输入8：角度最小值
% 输入9：T截断符号函数
% 输入10：最大的距离
% 输入11：最大的方位角
% 输入12：最大的俯仰角
% 输出1：TSDF地图
% 输出2：W TSDF权重

%           示例场景
% ｜ 0  0                  0|
% ｜_______                _|         
% ｜\ +1  | -1  -1  -1  -1| |
% ｜ \    |  0   0   0   0| |
% ｜  \   |_______________| |
% ｜   \   +1  +1   +1 /    |
% ｜    \             /     |
% ｜     \           /      |
% ｜      \         /       |
% ｜       \       /        |
% ｜        \     /         |
% ｜         \   /          |
% ｜           o 无人机      |
% ｜________________________|

function [TSDF, W] = GetTSDF(map, TSDFmap, weight, ...
    current_location, azimuth, ...
    elevation, dr, da, T, ...
    maximum_distance, maximum_azimuth, maximum_elevation)
    if nargin == 8
        T = 5;                             % 网格为5 （5 x dr）
        maximum_distance = 30 / dr;        % 30 meter
        maximum_azimuth = 45 / 180 * pi;   % FOV 90 degree
        maximum_elevation = 30 / 180 * pi; % FOV 60 degree
    end
    % 先找到障碍物位置
    map_define;
    % 找到建筑物的位置
    sx = size(map, 1);
    sy = size(map, 2);
    sz = size(map, 3);
    
    % 初始化TSDF地图
    % 以及TSDF权重
    map_TSDF    = ones(sx, sy, sz) * TSDF_NON;
    weight_TSDF = zeros(sx, sy, sz);
    
    % 这里直接计算出障碍物的位置，
    % 如果没有，全1
    for aa = -maximum_azimuth:da:maximum_azimuth
        for ee = -maximum_elevation:da:maximum_elevation
            % 遍历角度
            azi          = aa + azimuth;
            ele          = ee + elevation;
            obstacle_loc = [0, 0, 0]; % 当前角度的障碍物坐标
            rrr          = -1;        % 记录障碍物的位置
            for rr = 0:dr:maximum_distance
                zz   = rr * sin(ele);
                xxyy = rr * cos(ele);
                xx   = xxyy * cos(azi);
                yy   = xxyy * sin(azi);
                % 计算观测的位置
                new_index = fix(current_location + [xx, yy, zz]);
                map_TSDF(new_index(1), new_index(2), new_index(3)) = 1;
                weight_TSDF(new_index(1), new_index(2), new_index(3)) = 1;
                
                if new_index(1) <= 0 || new_index(2) <= 0 || ... 
                        new_index(3) <= 0 || new_index(1) >= sx || ...
                        new_index(2) >= sy || new_index(3) >= sz
                    continue;
                else
                    % 记录obstacle的位置
                    if map(new_index(1), new_index(2), new_index(3)) == obstacle
                        map_TSDF(new_index(1), new_index(2), new_index(3)) = 0;
                        obstacle_loc = new_index; % 记录位置
                        rrr = rr;
                        break
                    end
                end
            end
                
            if rrr > 0
                % 从头遍历后续的位置
                for rr = rrr:dr:maximum_distance
                    zz   = rr * sin(ele);
                    xxyy = rr * cos(ele);
                    xx   = xxyy * cos(azi);
                    yy   = xxyy * sin(azi);
                    % 计算观测的位置
                    new_index = fix(current_location + [xx, yy, zz]);
                    sdf        = norm(new_index - obstacle_loc);
                    if sdf < T
                        map_TSDF(new_index(1), new_index(2), new_index(3)) = -sdf / T;
                    else
                        map_TSDF(new_index(1), new_index(2), new_index(3)) = -1;
                    end
                    weight_TSDF(new_index(1), new_index(2), new_index(3)) = 1;
                end

                % 然后遍历前面的位置
                for rr = rrr:-dr:0
                    zz   = rr * sin(ele);
                    xxyy = rr * cos(ele);
                    xx   = xxyy * cos(azi);
                    yy   = xxyy * sin(azi);
                    % 计算观测的位置
                    new_index = fix(current_location + [xx, yy, zz]);
                    sdf        = norm(new_index - obstacle_loc);
                    if sdf < T
                        map_TSDF(new_index(1), new_index(2), new_index(3)) = sdf / T;
                    else
                        break
                    end
                end
            end
        end
    end
    
    % 
    % 利用 map_TSDF 以及 weight_TSDF
    % 融合TSDFmap以及weight
    % 
    
    % 预分配数据
    TSDF = TSDFmap;
    W    = weight;
    
    % 遍历网格 并且融合
    for xx = 1:sx
        for yy = 1:sy
            for zz = 1:sz
                if weight_TSDF(xx, yy, zz) == 0
                    continue; % 没有目标
                end
                
                W(xx, yy, zz) = weight(xx, yy, zz) + ...
                    weight_TSDF(xx, yy, zz);
                if TSDFmap(xx, yy, zz) == TSDF_NON
                    TSDF(xx, yy, zz) = map_TSDF(xx, yy, zz); % 初始化
                else
                    TSDF(xx, yy, zz) = (weight(xx, yy, zz) * TSDFmap(xx, yy, zz) + ...
                        weight_TSDF(xx, yy, zz) * map_TSDF(xx, yy, zz)) / W(xx, yy, zz);
                end
            end
        end
    end
    
end