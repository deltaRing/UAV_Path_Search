% ��ʼ�� Dstar ״̬ 
% ����1��xx ״̬��X�������
% ����2��yy ״̬��Y�������
% ����3��zz ״̬��Z�������
% �����״̬�ڵ� state
function state = init_state(xx, yy, zz)
   map_define;
   % ���ض���
   state.x = xx;
   state.y = yy;
   state.z = zz;
   state.parent = [];
   state.state = empty;
   state.t = new;
   state.h = 0;
   state.k = 0;
end

