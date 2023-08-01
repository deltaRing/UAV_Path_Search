% �����״̬��openList����
% ����1��Dstar D*�㷨��Ҫclass
% ����2��state ״̬
% ����3��h_new Hֵ
% ���1��Dstar
function Dstar = insert_state(Dstar, state, h_new)
    % ���е�ͼ��ʼ��
    map_define;
    if state.t == new
        state.k = h_new;
    elseif state.t == open_
        state.k = min([state.k, h_new]);
    elseif state.t == close_
        state.k = min([state.h, h_new]);
    end
    
    state.h = h_new;
    state.t = open_;
    sX = state.x; sY = state.y; sZ = state.z;
    
    % �����б������е���ֵ���б��� ȷ��û���ظ����б�
    for ii = 1:length(Dstar.openList)
        s = Dstar.openList(ii);
        if s.x == sX && s.y == sY && s.z == sZ
            % ����state
            Dstar.openList(ii) = state;
            Dstar.states(sX, sY, sZ) = state;
            return;
        end
    end
    
    Dstar.states(sX, sY, sZ) = state;
    Dstar.openList = [Dstar.openList state];
end
