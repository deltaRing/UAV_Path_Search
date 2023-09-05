% ��̬��Ѱ·����ʾ
% ����1��Dstar·��
% ����2��Dstar��ֹ·��
% ����3���������Ľ�����
% ����4�������ﲼ��
% ����5����ͼ

function dynamic_path_search(Dpath, Dend, searchBuilding, buildings, map_size)
    iters = length(Dpath); % �õ�֡��
    
    sx = map_size(1);
    sy = map_size(2);
    sz = map_size(3);
    
    % ��ʼ�� ��ͼ����
    map_define;
    % ������һ֡��·���㣨ƽ��B�������ߣ�
    last_point = [];
    
    for ii = 1:iters
        Dp = Dpath{ii}{1};
        De = Dend(ii);
        sB = searchBuilding{ii}{1};
        
        % ��ʼ�� ��ͼ
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
            fprintf("������̫�� �޷��Ż� ����\n");
            continue;
        end
        last_point = Dp(De,:);
        
        % �������˻�
        for ii = 1:length(Qopt)-10
            figure(10010)
            hold off
            % ���ƽ�����
            for iii = 1:size(buildings, 1)
                plot_building(buildings(iii,:));
            end
            hold on
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
            plot_camera(current_location, azimuth, elevation) % �������
            
            axis([0 sx + 5 0 sy + 5 0 sz])
            
            % pause(0.01); % ��ͣ10ms
            view([0 75]) % Y-Z
            drawnow
        end
        
        
    end

end