% 设置建筑物
% 输入1：原始的地图 map_before
% 输入2：建筑物起始坐标X PointX
% 输入3：建筑物起始坐标Y PointY
% 输入4：建筑物起始坐标Z PointZ
% 输入5：建筑物大小X     SizeX
% 输入6：建筑物大小Y     SizeY
% 输入7：建筑物大小Z     SizeZ
% 输入8：建筑物的值      Value
% 输出1：设置了建筑物之后的地图 Map_after
% 输出2：建筑物之间的顶点      Map_vertex
function [map_after, map_vertex] = set_building(map_before, ...
    pointx, pointy, pointz, ...
    sizex, sizey, sizez, ...
    value)
    map_after = map_before;
    map_vertex = [];
    
    % 规划建筑物的起始坐标 以及结束坐标
    pointx_s = pointx - sizex / 2;
    pointx_e = pointx + sizex / 2;
    pointy_s = pointy - sizey / 2;
    pointy_e = pointy + sizey / 2;
    pointz_s = pointz - sizez / 2;
    pointz_e = pointz + sizez / 2;
    
    % 检测建筑物是否在布局内部
    if pointx_s < 1, pointx_s = 1; end
    if pointy_s < 1, pointy_s = 1; end
    if pointz_s < 1, pointz_s = 1; end
    if pointx_e > size(map_before, 1), pointx_e = size(map_before, 1); end
    if pointy_e > size(map_before, 2), pointy_e = size(map_before, 2); end
    if pointz_e > size(map_before, 3), pointz_e = size(map_before, 3); end
    if pointx_s > pointx_e, pointx_s = pointx_e - 1; end
    if pointy_s > pointy_e, pointy_s = pointy_e - 1; end
    if pointz_s > pointz_e, pointz_s = pointz_e - 1; end
    
    % 生成建筑物
    map_vertex = [pointx_s pointx_e pointy_s pointy_e pointz_s pointz_e];
    map_after(pointx_s:pointx_e,pointy_s:pointy_e,pointz_s:pointz_e) = value;
end