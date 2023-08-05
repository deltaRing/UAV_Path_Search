% ������ײϵ��
% ����1��·����Q (1 x 3)
% ����2���ϰ��ﶥ��P (N x 3)
% ����3���ϰ���ԭ�еĶ�������V (N x 3)
% ���1��Jc ��ײ��ͷ�
function Jc = compute_slam(Q, P, V)
    Sf   = 3.0;
    Jc   = 0.0;
    % ��ײ��ͷ���ʼ��
    for ii = 1:size(P, 2)
        Obs = P{ii}; % �۲⵽���ϰ���
        vObs = V{ii};
        for jj = 1:size(P{ii}, 1)
            Dij = -(Q(ii,:) - Obs(jj, :)) * vObs(jj, :)'; % �������
            Cij = Sf - Dij;            % ����Cost
            Jcij = 0;
            if Cij <= 0
                % ����ʧ
            elseif Cij > 0.0 && Cij <= Sf
                Jcij = Cij^3;
            elseif Cij > Sf
               Jcij = 3 * Sf * Cij^2 - 3 * Sf^2 * Cij + Sf^3;
            end
            Jc = Jc + Jcij;
            % Cost����
        end
    end
end

