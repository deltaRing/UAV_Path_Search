% 检查节点
% 输入：1 当前节点
% 输入：2 期望节点
% 输出：1 相同：1 不相同：0
function result = checkNode(x, y)
    if isempty(x) || isempty(y)
        result = 0;
        return
    end

    if x.x == y.x && x.y == y.y && x.z == y.z
        result = 1;
    else
        result = 0;
    end
end