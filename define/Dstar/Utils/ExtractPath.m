% 提取路径
% 输入1：某个起始节点
% 输入2：记录多少个节点
% 输出1：该节点到最后终点的路径规划
function Q = ExtractPath(startState, numIndex)
    if nargin == 1, numIndex = 8; end
    map_define;
    % 记录所有的状态
    Q = [];
    % 新状态
    newState = startState;
    index    = 0;
    while newState.state ~= destination
        if index > numIndex, break; end
        Q = [Q; newState.x newState.y newState.z];
        newState = newState.parent;
        index = index + 1;
    end
    % 遍历路线找到终点
end

