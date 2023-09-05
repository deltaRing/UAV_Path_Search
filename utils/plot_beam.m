% 绘制扫描波束
% 输入1： Position X,Y,Z 扫描的波束位置
% 输入2： Scanning 扫描位置
% 输入3： 地图（如果地图有碰撞，则自动纠正扫描波束）
% 可选输入1：R 扫描半径
% 可选输入2：color 颜色

function plot_beam(Position, Scanning, map, R, color)

if nargin == 3
    color = 'g';
    R     = 0.5;
end

% 运行地图定义
map_define;

% 差异向量
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

%绘制圆锥底面
hold on;
EndPlate1=fill3(x(1,:),y(1,:),z(1,:),'r');
Cylinder=mesh(x,y,z);
%计算圆锥体旋转的角度
unit_V=[0 0 1];
angle_X1X2=acos(dot( unit_V,(Scanning - Position) )/( norm(unit_V)*norm(Scanning - Position)) )*180/pi;
%计算旋转轴
axis_rot=cross(unit_V,(Scanning - Position));
%将圆锥体旋转到期望方向
if angle_X1X2~=0 % Rotation is not needed if required direction is along X
    rotate(Cylinder,axis_rot,angle_X1X2,[0 0 0])
    rotate(EndPlate1,axis_rot,angle_X1X2,[0 0 0])
end
%将圆锥体和底面挪到期望的位置
set(EndPlate1,'XData',get(EndPlate1,'XData')+Position(1))
set(EndPlate1,'YData',get(EndPlate1,'YData')+Position(2))
set(EndPlate1,'ZData',get(EndPlate1,'ZData')+Position(3))
set(Cylinder,'XData',get(Cylinder,'XData')+Position(1))
set(Cylinder,'YData',get(Cylinder,'YData')+Position(2))
set(Cylinder,'ZData',get(Cylinder,'ZData')+Position(3))
% 设置圆锥体的颜色
set(Cylinder,'FaceColor',color)
set(EndPlate1,'FaceColor',color)
set(Cylinder,'EdgeAlpha',0)
set(EndPlate1,'EdgeAlpha',0)

end