function [A_d, B_d] = discreteEquivAandB(A_c, B_c, Ts)
%discreteEquivAandB converts continuous time A and B matrices into discrete
%time equivalents
    func = @(t) expm(A_c.*t);
    A_d = func(Ts);
    
    T_tmp = linspace(0,Ts, 5000);
    update = zeros(length(A_c));
    
    for i = 1:length(T_tmp)
        tmp = func(T_tmp(i));
        update = update + T_tmp(1+1).*tmp;
    end
    
    B_d = update*B_c;
end

