% ����������Դ���۵�
% ������Դ1����ײ�����Դ��� 
% ������Դ2������ѧ���ۣ��ٶȡ����ٶȡ��Ӽ��ٶȣ�
% ������Դ3�������Դ���
% ����1�� ·��          Q (M x 3)
% ����2�� Q�������ϰ��� P  (M x N x 3)
% ����3�� Q�������ϰ�������Ӧ������ V (M x N x 3)
% ���1�� ���еĴ���    (float Number)
function cost = compute_cost(Q, P, V)
    cost = 2.0 * compute_dynamic(Q) + 50.0 * compute_slam(Q,P,V) + 5.0 * compute_smooth(Q);
end

