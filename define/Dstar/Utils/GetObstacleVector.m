% �����ϰ��ﵽ·���������
% ����1����ͼMap
% ����2��ԭ��·��Path (should be N x 3)
% ����3����̽�⵽���ϰ������� (should be N x 3)
% ���1��P ������Ŀ��Ƶ�   (Ӧ����һ��)   M x (? x 3)
% ���2��V ��������         (Ӧ����һ��)   M x (? x 3)
% ���3��Q ·��
% ���4������
function [P, V, Q, D] = GetObstacleVector(Map, Path, NewObstacle)
    map_define;
    P = []; V = []; Q = []; D = [];
    % ���·�����СС��3 �޽������
    if size(Path, 1) <= 3, return; end
    % ��ǰ����������
    index = 1;
    % ���ص�ͼ����
    for pp = 1:size(Path, 1)
        px = Path(pp, 1); py = Path(pp, 2); pz = Path(pp, 3);
        if Map(px, py, pz) ~= obstacle
            continue
            % ��ҪѰ�ҵ��ϰ���Ľڵ�
        elseif Map(px, py, pz) == destination
            break;
        end
        % ·���õ������ϰ�����
        Q_t = []; V_t = []; P_t = []; D_t = [];
        
        for ppp = 1:size(NewObstacle, 1)
            ppx = NewObstacle(ppp, 1); 
            ppy = NewObstacle(ppp, 2);
            ppz = NewObstacle(ppp, 3);
            
            Vpp = [ppx - px ppy - py ppz - pz]; 
            Vpp = Vpp / norm(Vpp);
            dpp = [ppx - px ppy - py ppz - pz] * Vpp';
            if dpp < 0
                continue;
            end
            Q_t = [Q_t; px py pz];
            P_t = [P_t; ppx ppy ppz];
            V_t = [V_t; Vpp(1) Vpp(2) Vpp(3)];
            D_t = [D_t; dpp];
        end
        P{index} = P_t;
        V{index} = V_t;
        Q{index} = Q_t;
        D{index} = D_t;
        index = index + 1;
    end
    
end

