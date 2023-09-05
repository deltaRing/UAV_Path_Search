close all
clear all
clc

addpath map
addpath utils
addpath define
addpath define/Dstar
addpath define/Dstar/Utils
addpath define/Dstar/State
addpath define/Dstar/Dstar_algorithm
addpath algorithm
addpath data_structure

addpath drone
addpath drone/drone_kinect
addpath drone/path_opt

% 运行定义
map_define;

dx = 5;
dy = 5;
dz = 5;
da = 0.01;

% 地图大小
size_x = 100; size_y = 100; size_z = 100;
% 建筑物的布局
point_x_list = [10, 30, 50, 10, 70, 70, 80, 50];
point_y_list = [10, 15, 20, 60, 50, 10, 85, 60];
point_z_list = [10, 15, 25, 25, 45, 15, 40, 25]; % 中点
size_x_list  = [10, 10, 10, 30, 10, 10, 50, 10];
size_y_list  = [10, 30, 10, 30, 30, 30, 10, 10];
size_z_list  = [20, 30, 50, 50, 90, 30, 80, 50];

%     /|\
%      |
%      | _____
%      |/|  /|
%      /_|_/ |
%      |_|_|_|_________\
%     /| |_|_|         /
%    / | / | /
%   /  |/__|/   build construction
% |/_     

figure(10000)
% 建立地图
map         = map_init(floor(size_x / dx), floor(size_y / dy), floor(size_z / dz), 0);
map_series  = []; 
% 建立TSDF初始值
TSDF        = map_init(floor(size_x / dx), floor(size_y / dy), floor(size_z / dz), TSDF_NON);
TSDF_weight = map_init(floor(size_x / dx), floor(size_y / dy), floor(size_z / dz), 0);

% 建筑布置
for ii = 1:length(point_x_list)
    [map, series] = set_building(map, point_x_list(ii) / dx, ...
        point_y_list(ii) / dy, point_z_list(ii) / dz, ...
        size_x_list(ii) / dx, size_y_list(ii) / dy, ...
        size_z_list(ii) / dz, obstacle);
    plot_building_2(series);
    map_series = [map_series; series];
end

% 初始化 Dstar 的状态值
states = init_state_lists(map);

axis([0 size_x / dx 0 size_y / dy 0 size_z / dz])

% 假设无人机当前位置：
current_location = fix([5 / dx, 5 / dy, 40 / dz]);
% 无人机期望结束位置：
end_location     = fix([95 / dx, 95 / dy, 10 / dz]);
azimuth = pi / 4;
elevation = 0;
[observed, vector] = get_view(map, current_location, ...
    azimuth, elevation, dx, da);

% 绘制观测结果
figure(10001)
plot_observed(observed) % 绘制观测到的点
plot_camera(current_location, azimuth, elevation) % 绘制相机

% 计算当前TSDF结果
[TSDF, TSDF_weight] = GetTSDF(map, TSDF, TSDF_weight, ...
    current_location, azimuth, elevation, ...
    dx, da);

% 绘制当前TSDF结果
figure(10002)
plot_TSDF(TSDF, 0.1)

% 绘制路径
figure(10003)
for ii = 1:size(map_series, 1)
    plot_building_2(map_series(ii,:));
end
hold on

% 初始化Dstar
Dstar = init_Dstar(map, states);
% 找到起始的位置以及结束的位置
start_state = states(current_location(1), current_location(2), current_location(3));
end_state   = states(end_location(1), end_location(2), end_location(3));
% 设置状态
start_state = set_state(start_state, drone);
end_state   = set_state(end_state, destination);
Dstar.states(current_location(1), current_location(2), current_location(3)) = ...
    start_state;
Dstar.states(end_location(1), end_location(2), end_location(3)) = ...
    end_state;
% 测试运行Dstar路径
Dstar = Dstar_run(Dstar, start_state, end_state);
plot_Dstar_path(Dstar, current_location)

% 测试实时更新数据
[path, azis, eles, QRecord, QRecordEnd, QoptRecord, ScannerRecord] = Dstar_realtime(map, start_state, end_state, azimuth, elevation, map_series);

dynamic_path_search(QRecord, QRecordEnd, ScannerRecord, map_series, [20 20 20]);
static_path_search(QRecord, QRecordEnd, ScannerRecord, map_series, map);