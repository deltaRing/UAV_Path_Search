% ��ȡ·��
% ����1��ĳ����ʼ�ڵ�
% ����2����¼���ٸ��ڵ�
% ���1���ýڵ㵽����յ��·���滮
function Q = ExtractPath(startState, numIndex)
    if nargin == 1, numIndex = 8; end
    map_define;
    % ��¼���е�״̬
    Q = [];
    % ��״̬
    newState = startState;
    index    = 0;
    while newState.state ~= destination
        if index > numIndex, break; end
        Q = [Q; newState.x newState.y newState.z];
        newState = newState.parent;
        index = index + 1;
    end
    % ����·���ҵ��յ�
end

