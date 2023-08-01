classdef State
    %STATE �����D star ״̬
    
    % ��ֵ����
    properties
        state = [];  % ��ͼ״̬
        parent = []; % �ϸ��ڵ�
        x = -1;      % Xλ��
        y = -1;      % Yλ��
        z = -1;      % Zλ��
        t = 0;       % ���� close ״̬ ���� open ״̬ ���� new ״̬
        h = 0;
        k = 0;
    end
    
    methods
        function obs = State(xx, yy, zz)
            map_define;
            % ���ض���
            obs.x = xx;
            obs.y = yy;
            obs.z = zz;
            obs.parent = [];
            obs.state = empty;
            obs.t = new;
            obs.h = 0;
            obs.k = 0;
        end
        
        % �õ�����ֵ �þ���ֵ����ŷ�Ͼ���ֱ�Ӿ�����
        function value = cost(obj, state)
            map_define;
            % ���ض���
            if obj.state == obstacle, value = MAX_VALUE; return; end
            value = norm([obj.x - state.x, obj.y - state.y, obj.z - state.z]);
        end
        
        % ���øõ�Ԫ��״̬
        function obj = set_state(obj, state)
            obj.state = state;
        end
    end
end
