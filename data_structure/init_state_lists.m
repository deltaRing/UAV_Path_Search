% ���ݵ�ͼ����ʼ��״̬�б�
% ����1����ͼ
% ���1��״̬�б�
function state_lists = init_state_lists(map)
    % �õ���ͼ��С
    sx = size(map, 1);
    sy = size(map, 2);
    sz = size(map, 3);
    
    % ��¼ÿ��״ֵ̬
    for xx = sx:-1:1
        for yy = sy:-1:1
            for zz = sz:-1:1
                state_lists(xx,yy,zz) = init_state(xx, yy, zz);
                state_lists(xx,yy,zz) = set_state(state_lists(xx,yy,zz), map(xx, yy, zz));
            end
        end
    end
end

