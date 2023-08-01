% Drone_Azimuth 通过PID 更新无人机的角度 
% 输入1：current_a   当前的角度
% 输入2：expected_a  期望的角度
% 输入3：p           比例
% 输入4：i           积分
% 输入5：d           微分
% 输出1：new_angle   新角度
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

