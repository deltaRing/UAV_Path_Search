% �����������ҹ۲�TSDF�ı仯
% �Լ�TSDF_weight�ı仯
% ����1��TSDF OR TSDF_weight
% ����2����Ҫ�۲��ֵ��������
function plot_TSDF(map, value)
    sx = size(map, 1);
    sy = size(map, 2);
    sz = size(map, 3);
    [xx, yy, zz] = meshgrid([1:sx]);

    for ii = 1:length(value)
        % ȡ����Ӧ��ֵ
        v = value(ii);
        isosurface(xx, yy, zz, map, v);
        hold on
    end
    axis([0 sx 0 sy 0 sz])
end

