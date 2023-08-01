% 执行 Dstar 函数
% 输入1：Dstar 主类
% 输入2：start_state 开始的状态
% 输入3：end_state   结束的状态
% 输出1：Dstar 主类
function Dstar = Dstar_run(Dstar, start_state, end_state)
    map_define;
    % 初始化地图定义

    Dstar = insert_state(Dstar, end_state, 0.0);
    while 1
        [~, Dstar] = process_state(Dstar);
        if Dstar.states(start_state.x, start_state.y, start_state.z).t == close_ 
            break
        end
    end

end

