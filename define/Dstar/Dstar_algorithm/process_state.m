% Dstar算法
% 处理状态的过程
% 输入1：Dstar算法 主要CLASS
% 输出1：KMIN
function [kmin, Dstar] = process_state(Dstar)
    map_define;
    % 地图定义
    x = min_state(Dstar);
    if isempty(x), kmin = -1; return; end
    k_old = get_kmin(Dstar);
    Dstar = remove_state(Dstar, x);
    
    Xxx = x.x; Xyy = x.y; Xzz = x.z;
    
    fprintf("--------state: location: %f, %f, %f \n K: %f, H:%f \n", x.x, ...
        x.y, x.z, x.k, x.h)
    
    if k_old < x.h
       states = get_neighbor([x.x, x.y, x.z], Dstar.map, Dstar.states);
       for ss = 1:length(states)
           y = states(ss);
           if y.h <= k_old && x.h > y.h + get_cost(x, y)
               x.parent = y;
               x.h      = y.h + get_cost(x, y);
               Dstar.states(Xxx, Xyy, Xzz) = x;
           end
       end
    elseif k_old == x.h
       states = get_neighbor([x.x, x.y, x.z], Dstar.map, Dstar.states);
       for ss = 1:length(states)
           y = states(ss);
           f1 = y.t == new;
           f2 = checkNode(y.parent, x);
           f3 = y.h ~= x.h + get_cost(x, y);
           f4 = ~checkNode(y.parent, x);
           f5 = y.h > x.h + get_cost(x, y);
           f6 = y.state ~= destination;
           if f1 || (f2 && f3) || (f4 && f5) && f6
               y.parent = x;
               Dstar = insert_state(Dstar, y, x.h + get_cost(x, y));
           end
       end
    else
        states = get_neighbor([x.x, x.y, x.z], Dstar.map, Dstar.states);
        for ss = 1:length(states)
            y = states(ss);
            f1 = y.t == new;
            f2 = checkNode(y.parent, x);
            f3 = y.h ~= x.h + get_cost(x, y);
            if f1 || (f2 && f3)
                y.parent = x;
                Dstar = insert_state(Dstar, y, x.h + get_cost(x, y));
            else
                f4 = ~checkNode(y.parent, x); 
                f5 = y.h > x.h + get_cost(x,y);
                if f4 && f5
                    Dstar = insert_state(Dstar, x, x.h);
                else
                    f7  = ~checkNode(y.parent, x);
                    f8  = x.h > y.h + get_cost(x, y);
                    f9  = y.t == close;
                    f10 = y.h > k_old;
                    if f7 && f8 && f9 && f10 
                        Dstar = insert_state(Dstar, y, y.h);
                    end
                end
            end
        end
    end
    
    kmin = get_kmin(Dstar); 
    Dstar = update_state_parent(Dstar);
end
