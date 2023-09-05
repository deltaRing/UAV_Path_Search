% ��̬��Ѱ·����ʾ
% ����1��Dstar·��
% ����2��Dstar��ֹ·��
% ����3���������Ľ�����
% ����4�������ﲼ��
% ����5����ͼ�ߴ�
% ����6����ͼ

function static_path_search(Dpath, Dend, searchBuilding, buildings, map)
    iters = length(Dpath); % �õ�֡��
    
    sx = size(map, 1);
    sy = size(map, 2);
    sz = size(map, 3);
    
    % ��ʼ�� ��ͼ����
    map_define;
    % ������һ֡��·���㣨ƽ��B�������ߣ�
    sB = searchBuilding{iters}{1};
    % all paths
    Paths = [];
    for ii = 1:iters
        Dp = Dpath{ii}{1};
        De = Dend(ii);
        Paths = [Paths; Dp(1:De, :)];
    end
    
     % ��ʼ�� ��ͼ
     map_observed = sB;
        
     if size(Paths, 1) > 3
         [P, V] = FindObstacleVectorDstar(Paths(2:end,:), map_observed);

         CostFunc = @(Q_)compute_cost(Q_, P, V);
         [Popt, Cost] = fminsearch(CostFunc, Paths(2:end,:));
         disp(Cost)
         Qopt = b_spline_generate([Paths(1,:); Popt]);
     else
         fprintf("������̫�� �޷��Ż� ����\n");
         return;
     end
     
     % �������˻�
     for ii = 1:length(Qopt)-10     
        figure(10010)
        % ���ƽ�����
        for iii = 1:size(buildings, 1)
            plot_building_2(buildings(iii,:));
        end
        % ����ɨ�赽�ĳ���
        plot_observed(map_observed) % ���ƹ۲⵽�ĵ�
            
        plot3(Qopt(:,1), Qopt(:,2), Qopt(:,3))
        scatter3(Popt(:,1), Popt(:,2), Popt(:,3),5, 'g', 'filled')
            
        current_location = Qopt(ii, :);
        next_location    = Qopt(ii + 1, :);
        loc_vector       = next_location - current_location;
        azimuth          = atan2(loc_vector(2), loc_vector(1));
        elevation        = atan2(norm([loc_vector(1) loc_vector(2)]), loc_vector(3));
            
        % �������
        %plot_camera(current_location, azimuth, elevation) % �������
        % �������˻�
        plot_drone(current_location)
        axis([0 sx + 1 0 sy + 1 0 sz])
        % �̶���Ұ
        plot_beam([current_location(1),current_location(2),0], ...
                current_location, map)
            
        % pause(0.01); % ��ͣ10ms
        view([45 75]) % Y-Z
        drawnow
        hold off
    end

end