% ִ�� Dstar ����
% ���߸��¹۲⵽��observed ��ͼ��Ŀ
% figure is included
% ����1����ͼ��ʵ״̬ map
% ����2����ʼ��λ�� start_state
% ����3��������λ�� end_state
% ����4�����˻���ǰ�ķ�λ�� azimuth
% ����5�����˻���ǰ�ĸ����� elevation
% ����6�����˻���ǰ�Ľ������� building
% ����7��������С��Ԫ         dx
% ����8����С�Ƕȵ�Ԫ         da
% ����9�������             max_detect_times
% ���1��·��                path
% ���2��ÿ��·����������λ��   azis
% ���3��ÿ��·��������������   eles

% 2023�� 8�� 31�� �������
% QRecord����¼Dstar·��
% QRecordEnd����¼Dstar������·��
% QoptRecord����¼���·��
% ScannerRecord�� ��¼�ڼ����������������ɨ�赽�Ľ��

function [path, azis, eles, QRecord, ...
    QRecordEnd, QoptRecord, ScannerRecord] = ...
    Dstar_realtime(map, ...
    start_state, end_state, ...
    azimuth, elevation, ...
    building, ...
    dx, da, max_detect_times)
    % Ĭ�ϲ���
    if nargin <= 6
        dx = 0.1;
        da = 0.01;
        max_detect_times = 5;
    end
    % ����ʱ��
    dt   = 0.1;
    % ��ʼ������ֵ
    path = [];
    azis = [];
    eles = [];

    % ��ʼ������
    iter          = 0;
    QRecord       = {};
    QoptRecord    = {};
    ScannerRecord = {};
    QRecordEnd    = [];
    
    figure(10004)
    % ���ƽ�����
    for ii = 1:size(building, 1)
        plot_building(building(ii,:));
    end
    hold on
    
    sx = size(map, 1);
    sy = size(map, 2);
    sz = size(map, 3);
    
    % ��ʼ�� ��ͼ����
    map_define;
    % ��ʼ�� ��ͼ
    map_observed = map_init(sx, ...
        sy, ...
        sz, ...
        unknown);
    % ����״̬
    start_state = set_state(start_state, drone);
    end_state   = set_state(end_state, destination);
    % ��ʼ��
    middle_state = start_state;
    
    while ~checkNode(middle_state, end_state)
        % ������������
        iter = iter + 1;
      
        current_location = [middle_state.x, ...
            middle_state.y ...
            middle_state.z];
        % ɨ������
        [observed, ~] = get_view(map, current_location, ...
                azimuth, elevation, dx, da);
        % �������
        index_ovserved = find(observed ~= unknown);
        map_observed(index_ovserved) = observed(index_ovserved);
            
        % ����ɨ�赽�ĳ���
        plot_observed(map_observed) % ���ƹ۲⵽�ĵ�
            
        % ��ʼ��״̬
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
        % ����״̬
        Dstar.states(middle_state.x, middle_state.y, middle_state.z) = ...
            middle_state;
        Dstar.states(end_state.x, end_state.y, end_state.z) = ...
            end_state;
        
        % ���ݵ�ǰobserved ����·��
        Dstar = Dstar_run(Dstar, middle_state, end_state);
        % ���״̬
        middle_state = Dstar.states(middle_state.x, ...
                                    middle_state.y, ...
                                    middle_state.z);
        
        % �ҵ�·��
        Path = ExtractPath(middle_state);
        % �ҵ�·�����ϰ���
        Q      = b_spline_generate(Path);
        [P, V] = FindObstacleVectorDstar(Path(2:end,:), map_observed);
        
        CostFunc = @(Q_)compute_cost(Q_, P, V);
        [Popt, Cost] = fminsearch(CostFunc, Path(2:end,:));
        disp(Cost)
        Qopt = b_spline_generate([Path(1,:); Popt]);
        
        QRecord{iter}       = {Path};
        QoptRecord{iter}    = {Qopt};
        ScannerRecord{iter} = {map_observed};
        end_iter      = 0; % ��¼�������ε����Ĵ���
        
        if ~isempty(Q) && ~isempty(Q)
            figure(10005)
            hold off
             % ���ƽ�����
            for ii = 1:size(building, 1)
                plot_building(building(ii,:));
            end
            plot3(Q(:,1), Q(:,2), Q(:,3))
            hold on
            scatter3(Path(:,1), Path(:,2), Path(:,3),5, 'p', 'filled')
            plot3(Qopt(:,1), Qopt(:,2), Qopt(:,3))
            scatter3(Popt(:,1), Popt(:,2), Popt(:,3),5, 'g', 'filled')

            % ��ʾ�ϰ���
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
            % ����·����
        end
        
        figure(10004)
        % ������
        detect_time = 0;               
        % ��������
        while middle_state.state ~= obstacle && ...
                middle_state.state ~= unknown
            middle_state_p = middle_state.parent; % 
            
            end_iter = end_iter + 1;
            %
            % �õ��Ƕ�
            %
            
            if ~isempty(middle_state_p)
                msx = middle_state.x; msy = middle_state.y; msz = middle_state.z;
                mspx = middle_state_p.x; mspy = middle_state_p.y; mspz = middle_state_p.z;
                
                % 
                % 1.�ҵ��Ƕ�
                % 
                % ��λ�Ǹ���
                delta_x = mspx - msx; delta_y = mspy - msy; delta_z = mspz - msz;
                expected_az = atan2(delta_y, delta_x);
                delta_xy = sqrt(delta_x^2 + delta_y^2);
                % �����Ǹ���
                expected_el = atan2(delta_z, delta_xy);
                % �Ƕȸ���
                azimuth   = expected_az;
                elevation = expected_el;
            else
                disp('Failed find destination')
                break;
            end
            
            % 2. ��¼λ��
            path = [path; current_location];
            azis = [azis; azimuth];
            eles = [eles; elevation];
            
            % ���µ�ǰλ��
            current_location = [middle_state.x, ...
                middle_state.y ...
                middle_state.z];
            % �������
            plot_camera(current_location, azimuth, elevation) % �������
            
             % ������ϰ������û̽�������
            if middle_state_p.state == obstacle || ...
                    middle_state_p.state == unknown || ...
                    detect_time > max_detect_times
                break
            end
            
            % ��������յ�
            if middle_state_p.state == destination
                msx = middle_state.x; msy = middle_state.y; msz = middle_state.z;
                mspx = middle_state_p.x; mspy = middle_state_p.y; mspz = middle_state_p.z;
                
                % ��λ�Ǹ���
                delta_x = mspx - msx; delta_y = mspy - msy; delta_z = mspz - msz;
                expected_az = atan2(delta_y, delta_x);
                delta_xy = sqrt(delta_x^2 + delta_y^2);
                % �����Ǹ���
                expected_el = atan2(delta_z, delta_xy);
                % �Ƕȸ���
                azimuth   = expected_az;
                elevation = expected_el;
                middle_state = middle_state_p;
                
                % ��¼λ��
                path = [path; middle_state_p.x, middle_state_p.y middle_state_p.z];
                azis = [azis; azimuth];
                eles = [eles; elevation];
                break
            end
            detect_time = detect_time + 1;
            middle_state = middle_state_p;
        end
        % ����·��
        plot_Dstar_path(Dstar, current_location)
        % 
        QRecordEnd = [QRecordEnd end_iter];
    end
    
end

