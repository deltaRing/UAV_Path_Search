% ����ı��Ľ��
% �ϰ��﷢���仯��ͬʱ����Ŀ�������ʼ����ƣ����������ϰ��﷢���仯�������·�����۱仯
% ����1��Dstar D*�㷨��Ҫclass
% ����2��state ״̬
% ���1��Dstar
function Dstar = modify(Dstar, state)
    Dstar = modify_cost(Dstar, state);
    while 1
        [kmin, Dstar] = process_state(Dstar);
        if kmin >= state.h
            break
        end
    end
end
