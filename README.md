# UAV_Path_Search
本工程将借助ESDF、TSDF、Dstar、B样条曲线、以及多项式路径来实现无人机路径规划

ESDF 不想写了，后续移植到ROS上。

# 已知BUG
MATLAB R2022A 或者更新的版本上 会出现：
针对：unknown 标量的错误（请修改map_define.m上以及所有使用到该脚本的函数 unknown 的变量名 建议修改为 unknown_ 即可）

# 已知问题 x 2
在ROS上 Vins-Fusion 会由于不限于分辨率太低等问题导致错误估计姿态导致无人机炸鸡

# 已知问题 x 3
PX4 -pi 到 pi的问题 还没有修改 看心情随缘修改
