% ����ɨ�貨��
% ����1�� Position X,Y,Z ɨ��Ĳ���λ��
% ����2�� Scanning ɨ��λ��
% ����3�� ��ͼ�������ͼ����ײ�����Զ�����ɨ�貨����
% ��ѡ����1��R ɨ��뾶
% ��ѡ����2��color ��ɫ

function plot_beam(Position, Scanning, map, R, color)

if nargin == 3
    color = 'g';
    R     = 0.5;
end

% ���е�ͼ����
map_define;

% ��������
diff_vector  = Position - Scanning;
delta_vector = diff_vector / 1000;
for ii = 1:1000
    tentative = ceil(Scanning + delta_vector * ii);
    if map(tentative(1), tentative(2), tentative(3)) == obstacle
        Position = Scanning + delta_vector * ii;
        break
    end
end


length_cyl=norm(Scanning - Position);
[x,y,z]=cylinder(linspace(R,0,50),100);
z=z*length_cyl;

%����Բ׶����
hold on;
EndPlate1=fill3(x(1,:),y(1,:),z(1,:),'r');
Cylinder=mesh(x,y,z);
%����Բ׶����ת�ĽǶ�
unit_V=[0 0 1];
angle_X1X2=acos(dot( unit_V,(Scanning - Position) )/( norm(unit_V)*norm(Scanning - Position)) )*180/pi;
%������ת��
axis_rot=cross(unit_V,(Scanning - Position));
%��Բ׶����ת����������
if angle_X1X2~=0 % Rotation is not needed if required direction is along X
    rotate(Cylinder,axis_rot,angle_X1X2,[0 0 0])
    rotate(EndPlate1,axis_rot,angle_X1X2,[0 0 0])
end
%��Բ׶��͵���Ų��������λ��
set(EndPlate1,'XData',get(EndPlate1,'XData')+Position(1))
set(EndPlate1,'YData',get(EndPlate1,'YData')+Position(2))
set(EndPlate1,'ZData',get(EndPlate1,'ZData')+Position(3))
set(Cylinder,'XData',get(Cylinder,'XData')+Position(1))
set(Cylinder,'YData',get(Cylinder,'YData')+Position(2))
set(Cylinder,'ZData',get(Cylinder,'ZData')+Position(3))
% ����Բ׶�����ɫ
set(Cylinder,'FaceColor',color)
set(EndPlate1,'FaceColor',color)
set(Cylinder,'EdgeAlpha',0)
set(EndPlate1,'EdgeAlpha',0)

end