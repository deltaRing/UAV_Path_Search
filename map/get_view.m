% ���ù���׷�ٵķ��� �õ���ǰ������һ�� �������ˣ�
% ����1����ͼ
% ����2����ǰ����
% ����3����λ��
% ����4��������
% ����5��������С����
% ����6���Ƕ���С�Ƕ�
% ����7����Զ����
% ����8�����λ��
% ����9���������
% ����1����������һ��
% ����2����ǰ��ָ�ķ���

%  �Ƕȶ���
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
    
    % ���Ʒ�������
    point_vector = [1, 0, 0]';
    % ��λ�ǵ���ת����
    Razi = [cos(azimuth) -sin(azimuth) 0;
        sin(azimuth) cos(azimuth) 0;
        0 0 1]; % ����Z��
    % �����ǵ���ת����
    Rele = [1 0 0;
        0 cos(elevation) -sin(elevation);
        0 sin(elevation) cos(elevation)]; % ����X��
    % ������ת����
    point_vector = Razi * point_vector;
    point_vector = Rele * point_vector;
    
    % ���ж���
    map_define;

    sx = size(map, 1);
    sy = size(map, 2);
    sz = size(map, 3);
    map_seen = ones(sx, sy, sz) * unknown;
    
    for aa = -maximum_azimuth:da:maximum_azimuth
        for ee = -maximum_elevation:da:maximum_elevation
            % �����Ƕ�
            azi = aa + azimuth;
            ele = ee + elevation;
            for rr = 0:dr:maximum_distance
                zz   = rr * sin(ele);
                xxyy = rr * cos(ele);
                xx   = xxyy * cos(azi);
                yy   = xxyy * sin(azi);
                % �������
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

