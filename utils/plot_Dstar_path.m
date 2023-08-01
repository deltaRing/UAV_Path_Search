% ����Dstar ���ɵ�·��
% ����1��Dstar D*�㷨����
% ����2��start_location ��ʼ��·��
function plot_Dstar_path(Dstar, start_location)
    % �ҵ���Ӧ��·�����
    sx = start_location(1);
    sy = start_location(2);
    sz = start_location(3);
    % ��¼����
    path_vertex = [sx, sy, sz];
    % ���ض���
    map_define;
    
    while Dstar.states(sx, sy, sz).state ~= destination
        p = Dstar.states(sx, sy, sz).parent;
        % ����ǲ���empty
        if isempty(p), return; end
        % ��¼λ��
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

