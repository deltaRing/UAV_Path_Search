% �������������
% ���룺1�����˻���ǰ��λ��
% ���룺2�����˻��ķ�λ�Ƕ�
% ���룺3�����˻��ĸ����Ƕ�
function plot_camera(current_location, azimuth, elevation)
    % ����ͷĬ�Ϲ۲⵽�����FOV�Ƕ� 
    pt1 = [1, 1 * sin(pi / 3), 1 * sin(pi / 6)]';
    pt2 = [1, -1 * sin(pi / 3), 1 * sin(pi / 6)]';
    pt3 = [1, 1 * sin(pi / 3), -1 * sin(pi / 6)]';
    pt4 = [1, -1 * sin(pi / 3), -1 * sin(pi / 6)]';
    
    % ��λ�ǵ���ת����
    Razi = [cos(azimuth) -sin(azimuth) 0;
        sin(azimuth) cos(azimuth) 0;
        0 0 1]; % ����Z��
    % �����ǵ���ת����
    Rele = [1 0 0;
        0 cos(elevation) -sin(elevation);
        0 sin(elevation) cos(elevation)]; % ����X��
    % ������ת����
    pt1 = Razi * pt1;
    pt1 = Rele * pt1;
    pt2 = Razi * pt2;
    pt2 = Rele * pt2;
    pt3 = Razi * pt3;
    pt3 = Rele * pt3;
    pt4 = Razi * pt4;
    pt4 = Rele * pt4;
    % �����߶�
    pt1 = current_location' + pt1 * 1;
    pt2 = current_location' + pt2 * 1;
    pt3 = current_location' + pt3 * 1;
    pt4 = current_location' + pt4 * 1;
    
    % ��������ͷ
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

