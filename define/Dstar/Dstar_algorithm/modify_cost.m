% ͨ��parent ״̬���ƴ���
% ����1��Dstar D*�㷨��Ҫclass
% ����2��x ״̬
% ���1��Dstar
function Dstar = modify_cost(Dstar, x)
    % ���е�ͼ����ű�
    map_define;
    
    if x.t == close_
        Dstar = insert_state(Dstar, x, x.parent.h + get_cost(x, x.parent));
    end
end
