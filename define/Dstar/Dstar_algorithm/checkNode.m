% ���ڵ�
% ���룺1 ��ǰ�ڵ�
% ���룺2 �����ڵ�
% �����1 ��ͬ��1 ����ͬ��0
function result = checkNode(x, y)
    if isempty(x) || isempty(y)
        result = 0;
        return
    end

    if x.x == y.x && x.y == y.y && x.z == y.z
        result = 1;
    else
        result = 0;
    end
end