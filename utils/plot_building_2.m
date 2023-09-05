% plot_building2 ���ƽ�����
% �����������ƽ�����

function plot_building_2(building_vertex)
    px_start = building_vertex(1);
    px_end   = building_vertex(2);
    py_start = building_vertex(3);
    py_end   = building_vertex(4);
    pz_start = building_vertex(5);
    pz_end   = building_vertex(6);

    % ��ϳɶ���
    vertex_1 = [px_start, py_start, pz_start];
    vertex_2 = [px_end, py_start, pz_start];
    vertex_3 = [px_start, py_end, pz_start];
    vertex_4 = [px_start, py_start, pz_end];
    vertex_5 = [px_end, py_end, pz_start];
    vertex_6 = [px_end, py_start, pz_end];
    vertex_7 = [px_start, py_end, pz_end];
    vertex_8 = [px_end, py_end, pz_end];
    
    point_sequence=[1,2,3,4]; %�����ӵ�˳��
    square_xyz_1 = [vertex_1;vertex_3;vertex_5;vertex_2]; %��ά����4x3����
    patch('Faces',point_sequence,'Vertices',square_xyz_1,'FaceColor','blue');
    hold on 
    square_xyz_2 = [vertex_1;vertex_2;vertex_6;vertex_4]; %��ά����4x3����
    patch('Faces',point_sequence,'Vertices',square_xyz_2,'FaceColor','blue');
    
    square_xyz_3 = [vertex_2;vertex_6;vertex_8;vertex_5]; %��ά����4x3����
    patch('Faces',point_sequence,'Vertices',square_xyz_3,'FaceColor','blue');
    
    square_xyz_4 = [vertex_4;vertex_6;vertex_8;vertex_7]; %��ά����4x3����
    patch('Faces',point_sequence,'Vertices',square_xyz_4,'FaceColor','blue');
    
    square_xyz_5 = [vertex_5;vertex_3;vertex_7;vertex_8]; %��ά����4x3����
    patch('Faces',point_sequence,'Vertices',square_xyz_5,'FaceColor','blue');
    
    square_xyz_6 = [vertex_1;vertex_4;vertex_7;vertex_3]; %��ά����4x3����
    patch('Faces',point_sequence,'Vertices',square_xyz_6,'FaceColor','blue');
    view(3)
end