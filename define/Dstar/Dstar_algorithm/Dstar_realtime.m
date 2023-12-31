% 执行 Dstar 函数
% 在线更新观测到的observed 地图项目
% figure is included
% 输入1：地图真实状态 map
% 输入2：开始的位置 start_state
% 输入3：结束的位置 end_state
% 输入4：无人机当前的方位角 azimuth
% 输入5：无人机当前的俯仰角 elevation
% 输入6：无人机当前的建筑布局 building
% 输入7：距离最小单元         dx
% 输入8：最小角度单元         da
% 输入9：检测间隔             max_detect_times
% 输出1：路径                path
% 输出2：每个路径的期望方位角   azis
% 输出3：每个路径的期望俯仰角   eles

% 2023年 8月 31日 新增输出
% QRecord：记录Dstar路径
% QRecordEnd：记录Dstar结束的路径
% QoptRecord：记录最佳路径
% ScannerRecord： 记录第几个迭代的情况下所扫描到的结果

function [path, azis, eles, QRecord, ...
    QRecordEnd, QoptRecord, ScannerRecord] = ...
    Dstar_realtime(map, ...
    start_state, end_state, ...
    azimuth, elevation, ...
    building, ...
    dx, da, max_detect_times)
    % 默认参数
    if nargin <= 6
        dx = 0.1;
        da = 0.01;
        max_detect_times = 5;
    end
    % 步进时间
    dt   = 0.1;
    % 初始化返回值
    path = [];
    azis = [];
    eles = [];

    % 初始化绘制
    iter          = 0;
    QRecord       = {};
    QoptRecord    = {};
    ScannerRecord = {};
    QRecordEnd    = [];
    
    figure(10004)
    % 绘制建筑物
    for ii = 1:size(building, 1)
        plot_building(building(ii,:));
    end
    hold on
    
    sx = size(map, 1);
    sy = size(map, 2);
    sz = size(map, 3);
    
    % 初始化 地图定义
    map_define;
    % 初始化 地图
    map_observed = map_init(sx, ...
        sy, ...
        sz, ...
        unknown);
    % 设置状态
    start_state = set_state(start_state, drone);
    end_state   = set_state(end_state, destination);
    % 初始化
    middle_state = start_state;
    
    while ~checkNode(middle_state, end_state)
        % 迭代次数自增
        iter = iter + 1;
      
        current_location = [middle_state.x, ...
            middle_state.y ...
            middle_state.z];
        % 扫描数据
        [observed, ~] = get_view(map, current_location, ...
                azimuth, elevation, dx, da);
        % 填充数据
        index_ovserved = find(observed ~= unknown);
        map_observed(index_ovserved) = observed(index_ovserved);
            
        % 绘制扫描到的场景
        plot_observed(map_observed) % 绘制观测到的点
            
        % 初始化状态
        states = init_state_lists(map_observed);
        Dstar = init_Dstar(map_observed, states);
        middle_state = states(current_location(1), ...
                                current_location(2), ...
                                current_location(3));
        end_state    = states(end_state.x, ...
                        end_state.y, ...
                        end_state.z);
        middle_state = set_state(middle_state, drone);
        end_state   = set_state(end_state, destination);
        % 更新状态
        Dstar.states(middle_state.x, middle_state.y, middle_state.z) = ...
            middle_state;
        Dstar.states(end_state.x, end_state.y, end_state.z) = ...
            end_state;
        
        % 根据当前observed 建立路径
        Dstar = Dstar_run(Dstar, middle_state, end_state);
        % 添加状态
        middle_state = Dstar.states(middle_state.x, ...
                                    middle_state.y, ...
                                    middle_state.z);
        
        % 找到路径
        Path = ExtractPath(middle_state);
        % 找到路径上障碍物
        Q      = b_spline_generate(Path);
        [P, V] = FindObstacleVectorDstar(Path(2:end,:), map_observed);
        
        CostFunc = @(Q_)compute_cost(Q_, P, V);
        [Popt, Cost] = fminsearch(CostFunc, Path(2:end,:));
        disp(Cost)
        Qopt = b_spline_generate([Path(1,:); Popt]);
        
        QRecord{iter}       = {Path};
        QoptRecord{iter}    = {Qopt};
        ScannerRecord{iter} = {map_observed};
        end_iter      = 0; % 记录结束本次迭代的次数
        
        if ~isempty(Q) && ~isempty(Q)
            figure(10005)
            hold off
             % 绘制建筑物
            for ii = 1:size(building, 1)
                plot_building(building(ii,:));
            end
            plot3(Q(:,1), Q(:,2), Q(:,3))
            hold on
            scatter3(Path(:,1), Path(:,2), Path(:,3),5, 'p', 'filled')
            plot3(Qopt(:,1), Qopt(:,2), Qopt(:,3))
            scatter3(Popt(:,1), Popt(:,2), Popt(:,3),5, 'g', 'filled')

            % 显示障碍物
            xxxx = []; yyyy = []; zzzz = [];
            for xxx = 1:size(map_observed, 1)
                for yyy = 1:size(map_observed, 2)
                    for zzz = 1:size(map_observed, 3)
                        if map_observed(xxx, yyy, zzz) == obstacle
                            xxxx = [xxxx; xxx];
                            yyyy = [yyyy; yyy];
                            zzzz = [zzzz; zzz];
                        end
                    end
                end
            end
            scatter3(xxxx, yyyy, zzzz, 5, 'r', 'filled') 
            % 绘制路径点
        end
        
        figure(10004)
        % 检测次数
        detect_time = 0;               
        % 运行数据
        while middle_state.state ~= obstacle && ...
                middle_state.state ~= unknown
            middle_state_p = middle_state.parent; % 
            
            end_iter = end_iter + 1;
            %
            % 得到角度
            %
            
            if ~isempty(middle_state_p)
                msx = middle_state.x; msy = middle_state.y; msz = middle_state.z;
                mspx = middle_state_p.x; mspy = middle_state_p.y; mspz = middle_state_p.z;
                
                % 
                % 1.找到角度
                % 
                % 方位角更新
                delta_x = mspx - msx; delta_y = mspy - msy; delta_z = mspz - msz;
                expected_az = atan2(delta_y, delta_x);
                delta_xy = sqrt(delta_x^2 + delta_y^2);
                % 俯仰角更新
                expected_el = atan2(delta_z, delta_xy);
                % 角度更新
                azimuth   = expected_az;
                elevation = expected_el;
            else
                disp('Failed find destination')
                break;
            end
            
            % 2. 记录位置
            path = [path; current_location];
            azis = [azis; azimuth];
            eles = [eles; elevation];
            
            % 更新当前位置
            current_location = [middle_state.x, ...
                middle_state.y ...
                middle_state.z];
            % 绘制相机
            plot_camera(current_location, azimuth, elevation) % 绘制相机
            
             % 如果是障碍物或者没探测的区域
            if middle_state_p.state == obstacle || ...
                    middle_state_p.state == unknown || ...
                    detect_time > max_detect_times
                break
            end
            
            % 如果到达终点
            if middle_state_p.state == destination
                msx = middle_state.x; msy = middle_state.y; msz = middle_state.z;
                mspx = middle_state_p.x; mspy = middle_state_p.y; mspz = middle_state_p.z;
                
                % 方位角更新
                delta_x = mspx - msx; delta_y = mspy - msy; delta_z = mspz - msz;
                expected_az = atan2(delta_y, delta_x);
                delta_xy = sqrt(delta_x^2 + delta_y^2);
                % 俯仰角更新
                expected_el = atan2(delta_z, delta_xy);
                % 角度更新
                azimuth   = expected_az;
                elevation = expected_el;
                middle_state = middle_state_p;
                
                % 记录位置
                path = [path; middle_state_p.x, middle_state_p.y middle_state_p.z];
                azis = [azis; azimuth];
                eles = [eles; elevation];
                break
            end
            detect_time = detect_time + 1;
            middle_state = middle_state_p;
        end
        % 绘制路径
        plot_Dstar_path(Dstar, current_location)
        % 
        QRecordEnd = [QRecordEnd end_iter];
    end
    
end

