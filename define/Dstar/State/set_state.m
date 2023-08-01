% 设置观测状态
% 输入1：原来的state节点
% 输入2：新设置的状态
% 输出1：新的state节点
function obs = set_state(obs, new_state)
    obs.state = new_state;
end

