% 静态搜寻路线显示
% 输入1：Dstar路径
% 输入2：Dstar截止路径
% 输入3：搜索到的建筑物
% 输入4：建筑物布局
% 输入5：地图尺寸
% 输入6：地图

function static_path_search(Dpath, Dend, searchBuilding, buildings, map)
    iters = length(Dpath); % 得到帧数
    
    sx = size(map, 1);
    sy = size(map, 2);
    sz = size(map, 3);
    
    % 初始化 地图定义
    map_define;
    % 定义上一帧的路径点（平滑B样条曲线）
    sB = searchBuilding{iters}{1};
    % all paths
    Paths = [];
    for ii = 1:iters
        Dp = Dpath{ii}{1};
        De = Dend(ii);
        Paths = [Paths; Dp(1:De, :)];
    end
    
     % 初始化 地图
     map_observed = sB;
        
     if size(Paths, 1) > 3
         [P, V] = FindObstacleVectorDstar(Paths(2:end,:), map_observed);

         CostFunc = @(Q_)compute_cost(Q_, P, V);
         [Popt, Cost] = fminsearch(CostFunc, Paths(2:end,:));
         disp(Cost)
         Qopt = b_spline_generate([Paths(1,:); Popt]);
     else
         fprintf("数据量太少 无法优化 跳过\n");
         return;
     end
     
     % 绘制无人机
     for ii = 1:length(Qopt)-10     
        figure(10010)
        % 绘制建筑物
        for iii = 1:size(buildings, 1)
            plot_building_2(buildings(iii,:));
        end
        % 绘制扫描到的场景
        plot_observed(map_observed) % 绘制观测到的点
            
        plot3(Qopt(:,1), Qopt(:,2), Qopt(:,3))
        scatter3(Popt(:,1), Popt(:,2), Popt(:,3),5, 'g', 'filled')
            
        current_location = Qopt(ii, :);
        next_location    = Qopt(ii + 1, :);
        loc_vector       = next_location - current_location;
        azimuth          = atan2(loc_vector(2), loc_vector(1));
        elevation        = atan2(norm([loc_vector(1) loc_vector(2)]), loc_vector(3));
            
        % 绘制相机
        %plot_camera(current_location, azimuth, elevation) % 绘制相机
        % 绘制无人机
        plot_drone(current_location)
        axis([0 sx + 1 0 sy + 1 0 sz])
        % 固定视野
        plot_beam([current_location(1),current_location(2),0], ...
                current_location, map)
            
        % pause(0.01); % 暂停10ms
        view([45 75]) % Y-Z
        drawnow
        hold off
    end

end