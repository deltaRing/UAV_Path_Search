% 利用光线追踪的方法 得到当前所见的一切 （利用了）
% 输入1：地图
% 输入2：当前坐标
% 输入3：方位角
% 输入4：俯仰角
% 输入5：网格最小距离
% 输入6：角度最小角度
% 输入7：最远距离
% 输入8：最大方位角
% 输入9：最大俯仰角
% 返回1：所见到的一切
% 返回2：当前所指的方向

%  角度定义
%    Z
%    /\
%    |    0 degree
%    |    | Elevation
%    |    |  /  0 degree
%    |    |^/  Azimuth
%    |     /_)_____
%    |   Original Point
%    |____________________> Y
%   /
%  /    
%|/_
% X

function [map_seen, point_vector] = get_view(map, current_location, azimuth, ...
    elevation, dr, da, maximum_distance, maximum_azimuth, maximum_elevation)
    if nargin == 6
        maximum_distance = 30 / dr;        % 30 meter
        maximum_azimuth = 60 / 180 * pi;   % FOV 120 degree
        maximum_elevation = 60 / 180 * pi; % FOV 120 degree
    end
    
    % 绘制方向向量
    point_vector = [1, 0, 0]';
    % 方位角的旋转矩阵
    Razi = [cos(azimuth) -sin(azimuth) 0;
        sin(azimuth) cos(azimuth) 0;
        0 0 1]; % 绕着Z轴
    % 俯仰角的旋转矩阵
    Rele = [1 0 0;
        0 cos(elevation) -sin(elevation);
        0 sin(elevation) cos(elevation)]; % 绕着X轴
    % 设置旋转矩阵
    point_vector = Razi * point_vector;
    point_vector = Rele * point_vector;
    
    % 运行定义
    map_define;

    sx = size(map, 1);
    sy = size(map, 2);
    sz = size(map, 3);
    map_seen = ones(sx, sy, sz) * unknown;
    
    for aa = -maximum_azimuth:da:maximum_azimuth
        for ee = -maximum_elevation:da:maximum_elevation
            % 遍历角度
            azi = aa + azimuth;
            ele = ee + elevation;
            for rr = 0:dr:maximum_distance
                zz   = rr * sin(ele);
                xxyy = rr * cos(ele);
                xx   = xxyy * cos(azi);
                yy   = xxyy * sin(azi);
                % 计算相对
                map_select = fix(current_location + [xx, yy, zz]);
                
                if map_select(1) <= 0 || map_select(2) <= 0 || ... 
                        map_select(3) <= 0 || map_select(1) > sx || ...
                        map_select(2) > sy || map_select(3) > sz
                    continue;
                else
                    map_seen(map_select(1), map_select(2), map_select(3)) = ...
                        map(map_select(1), map_select(2), map_select(3));
                    if map(map_select(1), map_select(2), map_select(3)) == obstacle
                        break
                    end
                end
            end
        end
    end
    
end

