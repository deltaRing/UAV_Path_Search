% ��ͼ����
% obstacle       �ϰ��� 1
% drone          ���˻� 2
% empty          ��λ�� 0
% unknown        δ̽������ -1
% TSDF_NON TSDF  û�м����TSDF���� 3
% destination    �յ� 4
%

unknown     = -1;
empty       = 0;
obstacle    = 1;
drone       = 2;
TSDF_NON    = 3;
destination = 4;

% ״̬����
% NEW   0
% OPEN  1
% CLOSE 2
% MAX_VALUE 1e9 �����ϰ���
new       = 0;
open_      = 1;
close_     = 2;
MAX_VALUE = 1e9;

% ֵ����
Value_PATH_ = 1.0;                      % ·��
Value_PATH_CROSS_ACTION_ = sqrt(2);     % б������2
Value_PATH_CROSS_ACTION3_ = sqrt(3);    % б������3
Value_BARRIER_ = inf;                   % �ϰ���
Value_ROBOT_ = 0.0;                     % ���˻�
Value_DESTINATION_ = 0.0;               % �ص�
Value_ROUTE_ = 1.0;                     % ·��
Value_ROUTE_CROSS_ACTION_ = sqrt(2);    % б������2
Value_ROUTE_CROSS_ACTION3_ = sqrt(3);   % б������3
Value_UNKNOWN_ = 1.0;                   % δ֪����
