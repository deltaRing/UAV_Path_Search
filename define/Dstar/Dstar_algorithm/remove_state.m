% ɾ����Ӧ��state
% ����1��Dstar D*�㷨��Ҫclass
% ����2��state ״̬
% ���1��Dstar
function Dstar = remove_state(Dstar, state)
    map_define;
    % ���ص�ͼ����
    if state.t == open_
        state.t = close_;
    end
    
    for ii = 1:length(Dstar.openList)
        s = Dstar.openList(ii);
        if state.x == s.x && state.y == s.y && state.z == s.z
            Dstar.openList(ii) = [];
            Dstar.states(state.x, state.y, state.z) = state;
            break;
        end
    end
end