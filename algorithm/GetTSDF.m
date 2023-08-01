% ���TSDF��Ϣ
% ����1����ǰ��ͼ
% ����2����ǰTSDF��ͼ
% ����3����ǰȨ��
% ����4����ǰλ��
% ����5����ǰ��λ��
% ����6����ǰ������
% ����7��������Сֵ
% ����8���Ƕ���Сֵ
% ����9��T�ضϷ��ź���
% ����10�����ľ���
% ����11�����ķ�λ��
% ����12�����ĸ�����
% ���1��TSDF��ͼ
% ���2��W TSDFȨ��

%           ʾ������
% �� 0  0                  0|
% ��_______                _|         
% ��\ +1  | -1  -1  -1  -1| |
% �� \    |  0   0   0   0| |
% ��  \   |_______________| |
% ��   \   +1  +1   +1 /    |
% ��    \             /     |
% ��     \           /      |
% ��      \         /       |
% ��       \       /        |
% ��        \     /         |
% ��         \   /          |
% ��           o ���˻�      |
% ��________________________|

function [TSDF, W] = GetTSDF(map, TSDFmap, weight, ...
    current_location, azimuth, ...
    elevation, dr, da, T, ...
    maximum_distance, maximum_azimuth, maximum_elevation)
    if nargin == 8
        T = 5;                             % ����Ϊ5 ��5 x dr��
        maximum_distance = 30 / dr;        % 30 meter
        maximum_azimuth = 45 / 180 * pi;   % FOV 90 degree
        maximum_elevation = 30 / 180 * pi; % FOV 60 degree
    end
    % ���ҵ��ϰ���λ��
    map_define;
    % �ҵ��������λ��
    sx = size(map, 1);
    sy = size(map, 2);
    sz = size(map, 3);
    
    % ��ʼ��TSDF��ͼ
    % �Լ�TSDFȨ��
    map_TSDF    = ones(sx, sy, sz) * TSDF_NON;
    weight_TSDF = zeros(sx, sy, sz);
    
    % ����ֱ�Ӽ�����ϰ����λ�ã�
    % ���û�У�ȫ1
    for aa = -maximum_azimuth:da:maximum_azimuth
        for ee = -maximum_elevation:da:maximum_elevation
            % �����Ƕ�
            azi          = aa + azimuth;
            ele          = ee + elevation;
            obstacle_loc = [0, 0, 0]; % ��ǰ�Ƕȵ��ϰ�������
            rrr          = -1;        % ��¼�ϰ����λ��
            for rr = 0:dr:maximum_distance
                zz   = rr * sin(ele);
                xxyy = rr * cos(ele);
                xx   = xxyy * cos(azi);
                yy   = xxyy * sin(azi);
                % ����۲��λ��
                new_index = fix(current_location + [xx, yy, zz]);
                map_TSDF(new_index(1), new_index(2), new_index(3)) = 1;
                weight_TSDF(new_index(1), new_index(2), new_index(3)) = 1;
                
                if new_index(1) <= 0 || new_index(2) <= 0 || ... 
                        new_index(3) <= 0 || new_index(1) >= sx || ...
                        new_index(2) >= sy || new_index(3) >= sz
                    continue;
                else
                    % ��¼obstacle��λ��
                    if map(new_index(1), new_index(2), new_index(3)) == obstacle
                        map_TSDF(new_index(1), new_index(2), new_index(3)) = 0;
                        obstacle_loc = new_index; % ��¼λ��
                        rrr = rr;
                        break
                    end
                end
            end
                
            if rrr > 0
                % ��ͷ����������λ��
                for rr = rrr:dr:maximum_distance
                    zz   = rr * sin(ele);
                    xxyy = rr * cos(ele);
                    xx   = xxyy * cos(azi);
                    yy   = xxyy * sin(azi);
                    % ����۲��λ��
                    new_index = fix(current_location + [xx, yy, zz]);
                    sdf        = norm(new_index - obstacle_loc);
                    if sdf < T
                        map_TSDF(new_index(1), new_index(2), new_index(3)) = -sdf / T;
                    else
                        map_TSDF(new_index(1), new_index(2), new_index(3)) = -1;
                    end
                    weight_TSDF(new_index(1), new_index(2), new_index(3)) = 1;
                end

                % Ȼ�����ǰ���λ��
                for rr = rrr:-dr:0
                    zz   = rr * sin(ele);
                    xxyy = rr * cos(ele);
                    xx   = xxyy * cos(azi);
                    yy   = xxyy * sin(azi);
                    % ����۲��λ��
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
    % ���� map_TSDF �Լ� weight_TSDF
    % �ں�TSDFmap�Լ�weight
    % 
    
    % Ԥ��������
    TSDF = TSDFmap;
    W    = weight;
    
    % �������� �����ں�
    for xx = 1:sx
        for yy = 1:sy
            for zz = 1:sz
                if weight_TSDF(xx, yy, zz) == 0
                    continue; % û��Ŀ��
                end
                
                W(xx, yy, zz) = weight(xx, yy, zz) + ...
                    weight_TSDF(xx, yy, zz);
                if TSDFmap(xx, yy, zz) == TSDF_NON
                    TSDF(xx, yy, zz) = map_TSDF(xx, yy, zz); % ��ʼ��
                else
                    TSDF(xx, yy, zz) = (weight(xx, yy, zz) * TSDFmap(xx, yy, zz) + ...
                        weight_TSDF(xx, yy, zz) * map_TSDF(xx, yy, zz)) / W(xx, yy, zz);
                end
            end
        end
    end
    
end