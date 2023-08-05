% ����⻬��Ŀ
% ���룺Q [N x 3]
% ������⻬����� Js
function Js = compute_smooth(Q)
    path_size = size(Q, 1);
    
    dt = 0.333; % ���3m/s
    V = []; A = []; J = [];
    Js = 0.0;
    if path_size < 3
        return;
    end
    % ����ٶ�
    for qq = 1:path_size - 1
        V = [V; (Q(qq + 1, :) - Q(qq, :)) / dt];
    end
    % �����ٶ���ü��ٶ�
    for vv = 1:path_size - 2
        A  = [A; (V(vv + 1, :) - V(vv, :)) / dt];
        Js = Js + norm((V(vv + 1, :) - V(vv, :)) / dt);
    end
    % ���ݼ��ٶ���üӼ��ٶ�
    for aa = 1:path_size - 3
        J  = [J; (A(aa + 1, :) - A(aa, :)) / dt]; 
        Js = Js + norm((A(aa + 1, :) - A(aa, :)) / dt);
    end
end

