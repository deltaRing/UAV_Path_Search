% 初始化 Dstar 状态 
% 输入1：xx 状态在X轴的坐标
% 输入2：yy 状态在Y轴的坐标
% 输入3：zz 状态在Z轴的坐标
% 输出：状态节点 state
function state = init_state(xx, yy, zz)
   map_define;
   % 加载定义
   state.x = xx;
   state.y = yy;
   state.z = zz;
   state.parent = [];
   state.state = empty;
   state.t = new;
   state.h = 0;
   state.k = 0;
end

