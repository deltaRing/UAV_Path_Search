% ִ�� Dstar ����
% ����1��Dstar ����
% ����2��start_state ��ʼ��״̬
% ����3��end_state   ������״̬
% ���1��Dstar ����
function Dstar = Dstar_run(Dstar, start_state, end_state)
    map_define;
    % ��ʼ����ͼ����

    Dstar = insert_state(Dstar, end_state, 0.0);
    while 1
        [~, Dstar] = process_state(Dstar);
        if Dstar.states(start_state.x, start_state.y, start_state.z).t == close_ 
            break
        end
    end

end

