% ����ڵ�֮��Ĵ��ۺ���
% ����1����ǰ�ڵ� current_node
% ����2��Ԥ�ƽڵ� expected_node
% ���1������ֵ   value
function value = get_cost(current_node, expected_node)
    map_define;
    % ���ض���
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

