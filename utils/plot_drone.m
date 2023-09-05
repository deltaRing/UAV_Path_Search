% �������˻�
% plot_drone

% ����1��posi_d:  ���˻���ǰλ��
% ��ѡ����1��blade_r: ��Ҷ�뾶
% ��ѡ����2��frame_r: ���˻���ܰ뾶
% ��ѡ����3��frame_h: ���˻���ܸ߶�

function plot_drone(posi_d, blade_r, frame_r, frame_h)
    if nargin == 1
        blade_r = 0.25;
        frame_r = 0.05;
        frame_h = 0.3;
    end
    
    [Xb,Yb,Zb] = cylinder(blade_r);
    [Xd,Yd,Zd] = cylinder(frame_r);
    Zb = Zb * 0.1;
    Zd = Zd * -frame_h;
    
    offset_x = blade_r + frame_r / 2;
    offset_y = blade_r + frame_r / 2;
    
    % ���˻��ĸ���Ҷ��λ��
    Xb1 = Xb + offset_x; Yb1 = Yb + offset_y;
    Xb2 = Xb + offset_x; Yb2 = Yb - offset_y;
    Xb3 = Xb - offset_x; Yb3 = Yb + offset_y;
    Xb4 = Xb - offset_x; Yb4 = Yb - offset_y;
    % ���˻���λ��
    Dx  = posi_d(1);
    Dy  = posi_d(2);
    Dz  = posi_d(3);
    
    Blade1 = mesh(Xb1 + Dx, Yb1 + Dy, Zb + Dz);
    hold on
    Blade2 = mesh(Xb2 + Dx, Yb2 + Dy, Zb + Dz);
    Blade3 = mesh(Xb3 + Dx, Yb3 + Dy, Zb + Dz);
    Blade4 = mesh(Xb4 + Dx, Yb4 + Dy, Zb + Dz);
    Drone  = mesh(Xd + Dx, Yd + Dy, Zd + Dz);
    
    % ������ɫ
    set(Blade1,'FaceColor','r')
    set(Blade2,'FaceColor','r')
    set(Blade3,'FaceColor','r')
    set(Blade4,'FaceColor','r')
    set(Drone,'FaceColor','r')
    set(Blade1,'EdgeAlpha',0)
    set(Blade2,'EdgeAlpha',0)
    set(Blade3,'EdgeAlpha',0)
    set(Blade4,'EdgeAlpha',0)
    set(Drone,'EdgeAlpha',0)
 
    hold off
end