% 计算节点之间的代价函数
% 输入1：当前节点 current_node
% 输入2：预计节点 expected_node
% 输出1：代价值   value
function value = get_cost(current_node, expected_node)
    map_define;
    % 加载定义
    if current_node.state == obstacle
        value = inf; 
        return;
    end
    
    if expected_node.state == obstacle
        value = inf;
        return;
    end
    
    cost = norm([current_node.x - expected_node.x, ...
                    current_node.y - expected_node.y, ...
                    current_node.z - expected_node.z]);
                
    if current_node.state == empty
        value = Value_PATH_ * cost;
    elseif current_node.state == obstacle
        value = Value_BARRIER_;
    elseif current_node.state == destination
        value = Value_DESTINATION_;
    elseif current_node.state == drone
        value = Value_ROBOT_;
    elseif current_node.state == unknown
        value = Value_UNKNOWN_ * cost;
    else
        value = Value_PATH_ * cost;
    end
end

