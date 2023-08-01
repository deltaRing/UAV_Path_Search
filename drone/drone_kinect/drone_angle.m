% Drone_Azimuth ͨ��PID �������˻��ĽǶ� 
% ����1��current_a   ��ǰ�ĽǶ�
% ����2��expected_a  �����ĽǶ�
% ����3��p           ����
% ����4��i           ����
% ����5��d           ΢��
% ���1��new_angle   �½Ƕ�
function new_angle = drone_angle(current_a, expected_a, ...
    p_, i_, d_)
    if nargin == 2
        p_ = 1;
        i_ = 1;
        d_ = 1;
    end
    new_angle = current_a;
    error = expected_a - new_angle;
    it_ = 0;
    delta_ = 0;
    while abs(error) > 1e-4
        it_ = it_ + i_ * error;
        new_angle = p_ * error + i_ * it_ + d_ * delta_;
        last_err = error;
        error = expected_a - new_angle;
        delta_ = error - last_err;
    end
end

