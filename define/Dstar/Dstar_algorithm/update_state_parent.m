% ����ÿ��openlist״̬��parent
function Dstar = update_state_parent(Dstar)
    % ����openList
    for ii = 1:length(Dstar.openList)
        p = Dstar.openList(ii).parent;
        if isempty(p), continue; end
        pX = p.x; pY = p.y; pZ = p.z;
        Dstar.openList(ii).parent = Dstar.states(pX, pY, pZ);
    end
end