% 计算动力可行性
% 输入1：路径 Q
% 返回1：代价值
function Cost = compute_dynamic(Q)
    sSeqs = size(Q, 1); 
    Cost = 0;           % 代价数
    
    Cj = 3;             % Max 2m / s
    Cm = 10;
    dt = 0.3333333;         % 最快3m/s
    lambda = 1e-3;
    
    a1 = 0.5; a2 = 1.0;
    b1 = 0.5; b2 = 1.0;
    c1 = 2.0; c2 = 1.0;     % 多项式系数
    V = []; A = []; J = []; % 速度、加速度、加加速度
    % 求解速度
    for qq = 1:sSeqs - 1
        V_ = (Q(qq + 1, :) - Q(qq, :)) / dt;
        V = [V; V_];
        % 公式来自：https://zhuanlan.zhihu.com/p/366372048 （10）
        for qqq = 1:length(V_)
            Cr = V(qqq);
            if Cr < -Cj
                Cost = Cost + a1 * Cr^2 + b1 * Cr + c1;
            elseif Cr >= -Cj && Cr <-lambda * Cm
                Cost = Cost + (-lambda * Cm - Cr)^3;
            elseif -lambda * Cm <= Cr && Cr <= lambda * Cm
               
            elseif lambda * Cm < Cr && Cr <= Cj
                Cost = Cost + (Cr -lambda * Cm)^3;
            elseif Cr > Cj
                Cost = Cost + a2 * Cr^2 + b2 * Cr + c2;
            end
        end
    end
    % 根据速度求得加速度
    for vv = 1:sSeqs - 2
        A_ = (V(vv + 1, :) - V(vv, :)) / dt;
        A  = [A; A_];
        for vvv = 1:length(A_)
            Cr = A_(vvv);
            if Cr < -Cj
                Cost = Cost + a1 * Cr^2 + b1 * Cr + c1;
            elseif Cr >= -Cj && Cr <-lambda * Cm
                Cost = Cost + (-lambda * Cm - Cr)^3;
            elseif -lambda * Cm <= Cr && Cr <= lambda * Cm
               
            elseif lambda * Cm < Cr && Cr <= Cj
                Cost = Cost + (Cr -lambda * Cm)^3;
            elseif Cr > Cj
                Cost = Cost + a2 * Cr^2 + b2 * Cr + c2;
            end
        end
    end
    % 根据加速度求得加加速度
    for aa = 1:sSeqs - 3
        J_ = (A(aa + 1, :) - A(aa, :)) / dt;
        J  = [J; J_]; 
        for aaa = 1:length(A_)
            Cr = A_(aaa);
            if Cr < -Cj
                Cost = Cost + a1 * Cr^2 + b1 * Cr + c1;
            elseif Cr >= -Cj && Cr <-lambda * Cm
                Cost = Cost + (-lambda * Cm - Cr)^3;
            elseif -lambda * Cm <= Cr && Cr <= lambda * Cm
               
            elseif lambda * Cm < Cr && Cr <= Cj
                Cost = Cost + (Cr -lambda * Cm)^3;
            elseif Cr > Cj
                Cost = Cost + a2 * Cr^2 + b2 * Cr + c2;
            end
        end
    end
    % 顺便也把代价求解出来
end

