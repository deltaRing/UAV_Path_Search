% ��ʼ���µĵ�ͼ
% ���룺1 size_x X�ĳߴ�
% ���룺2 size_y Y�ĳߴ�
% ���룺3 size_z Z�ĳߴ�
% ���룺4 value  ��ʼ����ֵ
% �����1 ��ͼ
function map = map_init(size_x, size_y, size_z, value)
    map = ones([size_x, size_y, size_z]) * value;
end