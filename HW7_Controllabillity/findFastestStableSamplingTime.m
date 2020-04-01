function [T_fast] = findFastestStableSamplingTime(A_d, T_start)
%findFastestStableSamplingTime takes in the discretized A matrix (A_d) and
%a starting sampling period (T_start) then returns the fastest sampling 
%time before instability occurs due to thm 6.9 in the textbook

    eigVals = eig(A_d);
    isStable = 1;
    T_fast = T_start;

    while (T_fast > 1e-9) && isStable
        T_fast = T_fast*.1;

        for i = 2:length(eigVals)
            if real(eigVals(i-1)-eigVals(i)) == 0
                if mod(T_fast, (2*pi)/abs(imag(eigVals(i-1) - eigVals(i)))) == 0
                    isStable = 0;
                end
            end
        end
    end
    
    if T_fast <= 1e-9
        T_fast = 0;
    end
    
end

