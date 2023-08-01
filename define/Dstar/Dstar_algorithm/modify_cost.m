% 通过parent 状态递推代价
% 输入1：Dstar D*算法主要class
% 输入2：x 状态
% 输出1：Dstar
function Dstar = modify_cost(Dstar, x)
    % 运行地图定义脚本
    map_define;
    
    if x.t == close_
        Dstar = insert_state(Dstar, x, x.parent.h + get_cost(x, x.parent));
    end
end
