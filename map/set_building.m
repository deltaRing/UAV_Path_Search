% ���ý�����
% ����1��ԭʼ�ĵ�ͼ map_before
% ����2����������ʼ����X PointX
% ����3����������ʼ����Y PointY
% ����4����������ʼ����Z PointZ
% ����5���������СX     SizeX
% ����6���������СY     SizeY
% ����7���������СZ     SizeZ
% ����8���������ֵ      Value
% ���1�������˽�����֮��ĵ�ͼ Map_after
% ���2��������֮��Ķ���      Map_vertex
function [map_after, map_vertex] = set_building(map_before, ...
    pointx, pointy, pointz, ...
    sizex, sizey, sizez, ...
    value)
    map_after = map_before;
    map_vertex = [];
    
    % �滮���������ʼ���� �Լ���������
    pointx_s = pointx - sizex / 2;
    pointx_e = pointx + sizex / 2;
    pointy_s = pointy - sizey / 2;
    pointy_e = pointy + sizey / 2;
    pointz_s = pointz - sizez / 2;
    pointz_e = pointz + sizez / 2;
    
    % ��⽨�����Ƿ��ڲ����ڲ�
    if pointx_s < 1, pointx_s = 1; end
    if pointy_s < 1, pointy_s = 1; end
    if pointz_s < 1, pointz_s = 1; end
    if pointx_e > size(map_before, 1), pointx_e = size(map_before, 1); end
    if pointy_e > size(map_before, 2), pointy_e = size(map_before, 2); end
    if pointz_e > size(map_before, 3), pointz_e = size(map_before, 3); end
    if pointx_s > pointx_e, pointx_s = pointx_e - 1; end
    if pointy_s > pointy_e, pointy_s = pointy_e - 1; end
    if pointz_s > pointz_e, pointz_s = pointz_e - 1; end
    
    % ���ɽ�����
    map_vertex = [pointx_s pointx_e pointy_s pointy_e pointz_s pointz_e];
    map_after(pointx_s:pointx_e,pointy_s:pointy_e,pointz_s:pointz_e) = value;
end