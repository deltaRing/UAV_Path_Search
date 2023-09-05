% 动态搜寻路线显示
% 输入1：Dstar路径
% 输入2：Dstar截止路径
% 输入3：搜索到的建筑物
% 输入4：建筑物布局
% 输入5：地图

function dynamic_path_search(Dpath, Dend, searchBuilding, buildings, map_size)
    iters = length(Dpath); % 得到帧数
    
    sx = map_size(1);
    sy = map_size(2);
    sz = map_size(3);
    
    % 初始化 地图定义
    map_define;
    % 定义上一帧的路径点（平滑B样条曲线）
    last_point = [];
    
    for ii = 1:iters
        Dp = Dpath{ii}{1};
        De = Dend(ii);
        sB = searchBuilding{ii}{1};
        
        % 初始化 地图
        map_observed = sB;
        
        if size(Dp, 1) > 3
            
            if ~isempty(last_point)
                [P, V] = FindObstacleVectorDstar(Dp(2:De,:), map_observed);

                CostFunc = @(Q_)compute_cost(Q_, P, V);
                [Popt, Cost] = fminsearch(CostFunc, Dp(2:De,:));
                disp(Cost)
                Qopt = b_spline_generate([last_point; Dp(1,:); Popt]);
            else
                [P, V] = FindObstacleVectorDstar(Dp(2:De,:), map_observed);

                CostFunc = @(Q_)compute_cost(Q_, P, V);
                [Popt, Cost] = fminsearch(CostFunc, Dp(2:De,:));
                disp(Cost)
                Qopt = b_spline_generate([Dp(1,:); Popt]);
            end
        else
            fprintf("数据量太少 无法优化 跳过\n");
            continue;
        end
        last_point = Dp(De,:);
        
        % 绘制无人机
        for ii = 1:length(Qopt)-10
            figure(10010)
            hold off
            % 绘制建筑物
            for iii = 1:size(buildings, 1)
                plot_building(buildings(iii,:));
            end
            hold on
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
            plot_camera(current_location, azimuth, elevation) % 绘制相机
            
            axis([0 sx + 5 0 sy + 5 0 sz])
            
            % pause(0.01); % 暂停10ms
            view([0 75]) % Y-Z
            drawnow
        end
        
        
    end

end