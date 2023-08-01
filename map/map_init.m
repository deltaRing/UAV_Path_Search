% 初始化新的地图
% 输入：1 size_x X的尺寸
% 输入：2 size_y Y的尺寸
% 输入：3 size_z Z的尺寸
% 输入：4 value  初始化的值
% 输出：1 地图
function map = map_init(size_x, size_y, size_z, value)
    map = ones([size_x, size_y, size_z]) * value;
end