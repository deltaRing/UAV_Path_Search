% 绘制Dstar 生成的路径
% 输入1：Dstar D*算法主类
% 输入2：start_location 开始的路径
function plot_Dstar_path(Dstar, start_location)
    % 找到对应的路径起点
    sx = start_location(1);
    sy = start_location(2);
    sz = start_location(3);
    % 记录顶点
    path_vertex = [sx, sy, sz];
    % 加载定义
    map_define;
    
    while Dstar.states(sx, sy, sz).state ~= destination
        p = Dstar.states(sx, sy, sz).parent;
        % 检测是不是empty
        if isempty(p), return; end
        % 记录位置
        sx = p.x; sy = p.y; sz = p.z;
        path_vertex = [path_vertex; sx, sy, sz];
    end
    
    plot3(path_vertex(:,1), ...
        path_vertex(:,2), ...
        path_vertex(:,3))
    hold on
    scatter3(path_vertex(:,1), ...
        path_vertex(:,2), ...
        path_vertex(:,3))
end

