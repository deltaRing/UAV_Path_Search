% �õ�����
% ����1����ǰλ�� location
% ����2����ͼ maps
% ����3��״̬ states
% ����������¼��ֵ
function state_list = get_neighbor(location, map, states)
    % �õ���ͼ��С
    sx = size(map, 1);
    sy = size(map, 2);
    sz = size(map, 3);
    
    state_list = [];

    % ��������������
    for x = -1:1
        for y = -1:1
            for z = -1:1
                xx = location(1) + x;
                yy = location(2) + y;
                zz = location(3) + z;
                
                % ��������߽磬�ܾ�����
                if xx <= 0 || xx > sx
                    continue;
                end
                if yy <= 0 || yy > sy
                    continue;
                end
                if zz <= 0 || zz > sz
                    continue;
                end
                % ����Ǳ��� �ܾ�����
                if x == 0 && y == 0 && z == 0
                    continue;
                end
                state_list = [state_list states(xx, yy, zz)];
            end
        end
    end
end

