% ����Dstar������ ����B��������
% ����1�� Path ����DStar�㷨Ԥ�ȵõ���·��
% ����2�� dt
% ����3�� order ���׵�B��������
% ���4�� new_path ���µ�b����·��
function new_path = b_spline_generate(Path_point, dt, order)
    if nargin == 1
        dt = 0.01;
        order = 3;
    end
    % ���µ�B����·�������ֵ
    new_path = []; 
    n = size(Path_point, 1) - 1;
    % data ������Ŀ
    
    if n < order
        disp('Data size must be greater than order')
        return;
    end
    
    % ����ÿ��·����
    NodeVector = linspace(0, 1, n + order + 1); % �ڵ�ʸ��
    % t ���� 0 ---> 1
    for t = (order-1)/(n+order+1):dt:(n+2) / (n+order+1)
        % Ԥ�ȶ�������
        pts = [];
        Bs  = [];
        % ��������
        for tt = 0:1:n % �������һ������
            pt  = Path_point(tt + 1, :);
            B   = N(t, NodeVector, tt, order - 1);
            pts = [pts; pt];
            Bs  = [Bs B];
        end
        xxx = Bs * pts(:,1); yyy = Bs * pts(:,2); zzz = Bs * pts(:,3);
        new_path = [new_path; xxx yyy zzz];
    end 
end

% ���������
% ����1��t ��ǰ��t
% ����2��k ����
% ����3��m ����
% ���1��weight Ȩ��
function weight = N(t, NodeVector, k, m)
    if m == 0
        if t >= NodeVector(k+1) && t <= NodeVector(k+2)
            weight = 1; 
            return;
        else
            weight = 0;
            return;
        end
    else
        L1 = (NodeVector(k + m + 1) - NodeVector(k + 1));
        L2 = (NodeVector(k + m + 2) - NodeVector(k + 2));
        if L1 == 0, L1 = 1; end % �涨�� 0 / 0 = 0
        if L2 == 0, L2 = 1; end
        Bk_M_1   = N(t, NodeVector, k, m - 1);
        Bk_1_M_1 = N(t, NodeVector, k + 1, m - 1);
        weight = (t - NodeVector(k + 1)) / L1 * Bk_M_1 + ...
            (NodeVector(m + k + 2) - t) / L2 * Bk_1_M_1;
        return;
    end
end