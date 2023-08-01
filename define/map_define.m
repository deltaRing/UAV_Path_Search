% 地图定义
% obstacle       障碍物 1
% drone          无人机 2
% empty          空位置 0
% unknown        未探索区域 -1
% TSDF_NON TSDF  没有计算的TSDF区域 3
% destination    终点 4
%

unknown     = -1;
empty       = 0;
obstacle    = 1;
drone       = 2;
TSDF_NON    = 3;
destination = 4;

% 状态定义
% NEW   0
% OPEN  1
% CLOSE 2
% MAX_VALUE 1e9 堵塞障碍物
new       = 0;
open_      = 1;
close_     = 2;
MAX_VALUE = 1e9;

% 值定义
Value_PATH_ = 1.0;                      % 路径
Value_PATH_CROSS_ACTION_ = sqrt(2);     % 斜向行走2
Value_PATH_CROSS_ACTION3_ = sqrt(3);    % 斜向行走3
Value_BARRIER_ = inf;                   % 障碍物
Value_ROBOT_ = 0.0;                     % 无人机
Value_DESTINATION_ = 0.0;               % 重点
Value_ROUTE_ = 1.0;                     % 路线
Value_ROUTE_CROSS_ACTION_ = sqrt(2);    % 斜向行走2
Value_ROUTE_CROSS_ACTION3_ = sqrt(3);   % 斜向行走3
Value_UNKNOWN_ = 1.0;                   % 未知区域
