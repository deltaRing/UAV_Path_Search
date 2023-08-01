% 绘制相机的线条
% 输入：1、无人机当前的位置
% 输入：2、无人机的方位角度
% 输入：3、无人机的俯仰角度
function plot_camera(current_location, azimuth, elevation)
    % 摄像头默认观测到的最大FOV角度 
    pt1 = [1, 1 * sin(pi / 3), 1 * sin(pi / 6)]';
    pt2 = [1, -1 * sin(pi / 3), 1 * sin(pi / 6)]';
    pt3 = [1, 1 * sin(pi / 3), -1 * sin(pi / 6)]';
    pt4 = [1, -1 * sin(pi / 3), -1 * sin(pi / 6)]';
    
    % 方位角的旋转矩阵
    Razi = [cos(azimuth) -sin(azimuth) 0;
        sin(azimuth) cos(azimuth) 0;
        0 0 1]; % 绕着Z轴
    % 俯仰角的旋转矩阵
    Rele = [1 0 0;
        0 cos(elevation) -sin(elevation);
        0 sin(elevation) cos(elevation)]; % 绕着X轴
    % 设置旋转矩阵
    pt1 = Razi * pt1;
    pt1 = Rele * pt1;
    pt2 = Razi * pt2;
    pt2 = Rele * pt2;
    pt3 = Razi * pt3;
    pt3 = Rele * pt3;
    pt4 = Razi * pt4;
    pt4 = Rele * pt4;
    % 绘制线段
    pt1 = current_location' + pt1 * 1;
    pt2 = current_location' + pt2 * 1;
    pt3 = current_location' + pt3 * 1;
    pt4 = current_location' + pt4 * 1;
    
    % 绘制摄像头
    plot3([pt1(1) pt2(1)], [pt1(2) pt2(2)], [pt1(3) pt2(3)], 'r')
    plot3([pt2(1) pt3(1)], [pt2(2) pt3(2)], [pt2(3) pt3(3)], 'r')
    plot3([pt3(1) pt4(1)], [pt3(2) pt4(2)], [pt3(3) pt4(3)], 'r')
    plot3([pt4(1) pt1(1)], [pt4(2) pt1(2)], [pt4(3) pt1(3)], 'r')
    plot3([pt1(1) pt3(1)], [pt1(2) pt3(2)], [pt1(3) pt3(3)], 'r')
    plot3([pt2(1) pt4(1)], [pt2(2) pt4(2)], [pt2(3) pt4(3)], 'r')
    plot3([current_location(1) pt1(1)], [current_location(2) pt1(2)], [current_location(3) pt1(3)], 'r')
    plot3([current_location(1) pt2(1)], [current_location(2) pt2(2)], [current_location(3) pt2(3)], 'r')
    plot3([current_location(1) pt3(1)], [current_location(2) pt3(2)], [current_location(3) pt3(3)], 'r')
    plot3([current_location(1) pt4(1)], [current_location(2) pt4(2)], [current_location(3) pt4(3)], 'r')
end

